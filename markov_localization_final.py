
#!/usr/bin/env python3
"""
Markov localization on a road graph using sequences of observed traffic signs.

Idea
----
You provide a sequence of sign observations such as:

    stop -> priority -> crosswalk

The script:
1. Loads the GraphML road graph.
2. Finds all nodes that carry a traffic-sign label.
3. Builds a "sign-event graph":
      an edge A -> B exists if, starting from sign node A and driving forward,
      you can reach sign node B before reaching any other sign node.
4. Runs:
      - Forward filtering: current belief over possible sign nodes
      - Viterbi decoding: most likely sequence of sign nodes / route
5. Reconstructs the most likely full route on the original graph.

This is a practical HMM / Markov-localization style simulation for:
"Given the signs I see one after another, where am I likely to be?"
"""

from __future__ import annotations
import matplotlib
matplotlib.use('TkAgg')
import argparse
import json
import math
import os
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Dict, List, Set, Tuple, Optional
import matplotlib.pyplot as plt
import networkx as nx
from matplotlib.patches import Circle
from PIL import Image

from lxml import etree


GRAPHML_NS = "http://graphml.graphdrawing.org/xmlns"
NS = {"g": GRAPHML_NS}

# Normalize labels found in the file to cleaner names.
SIGN_ALIASES = {
    "roudabout": "roundabout",
    "roundabout": "roundabout",
    "endhighway": "endhighway",
    "endHighway": "endhighway",
    "highway": "highway",
    "parking": "parking",
    "crosswalk": "crosswalk",
    "priority": "priority",
    "stop": "stop",
}

# Friendly output names
DISPLAY_NAMES = {
    "roundabout": "roundabout",
    "endhighway": "endHighway",
    "highway": "highway",
    "parking": "parking",
    "crosswalk": "crosswalk",
    "priority": "priority",
    "stop": "stop",
}


@dataclass
class NodeInfo:
    node_id: str
    x: float
    y: float
    signs: Set[str]


class MarkovSignLocalizer:
    def __init__(self, graphml_path: str, p_hit: float = 0.92, p_miss: float = 0.02, p_skip: float = 0.10):
        self.graphml_path = graphml_path
        self.p_hit = p_hit
        self.p_miss = p_miss
        self.p_skip = p_skip

        self.nodes: Dict[str, NodeInfo] = {}
        self.adj: Dict[str, List[str]] = defaultdict(list)
        self.rev_adj: Dict[str, List[str]] = defaultdict(list)

        self.sign_nodes: List[str] = []
        self.sign_types: Set[str] = set()

        # sign_event_edges[a] = [b1, b2, ...]
        self.sign_event_edges: Dict[str, List[str]] = defaultdict(list)

        # transition_paths[(a, b)] = full node path on the original graph
        self.transition_paths: Dict[Tuple[str, str], List[str]] = {}

        self.transition_skips: Dict[Tuple[str, str], int] = {}
        self.sign_transition_probs: Dict[Tuple[str, str], float] = {}

        '''
        self._load_graphml()
        self._build_sign_event_graph()
        '''

    def _load_graphml(self) -> None:
        tree = etree.parse(self.graphml_path)
        root = tree.getroot()

        # Nodes
        for node in root.findall(".//g:node", NS):
            node_id = node.get("id")
            x = None
            y = None
            signs: Set[str] = set()

            for child in node:
                local = etree.QName(child).localname
                if local != "data":
                    continue
                key = child.get("key")
                text = (child.text or "").strip()

                if key == "d0":
                    try:
                        x = float(text)
                    except ValueError:
                        x = 0.0
                elif key == "d1":
                    try:
                        y = float(text)
                    except ValueError:
                        y = 0.0
                elif key and key.startswith("d_") and text.lower() == "true":
                    raw_label = key[2:]
                    normalized = SIGN_ALIASES.get(raw_label, raw_label.lower())
                    signs.add(normalized)
                    self.sign_types.add(normalized)

            self.nodes[node_id] = NodeInfo(
                node_id=node_id,
                x=0.0 if x is None else x,
                y=0.0 if y is None else y,
                signs=signs,
            )

        # Edges
        for edge in root.findall(".//g:edge", NS):
            src = edge.get("source")
            dst = edge.get("target")
            if src is None or dst is None:
                continue
            self.adj[src].append(dst)
            self.rev_adj[dst].append(src)

        self.sign_nodes = [nid for nid, info in self.nodes.items() if info.signs]

    def _build_sign_event_graph(self) -> None:

        max_skips  = 1
        sign_node_set = set(self.sign_nodes)

        # Reset all derived structures
        self.sign_event_edges.clear()
        self.transition_paths.clear()
        self.transition_skips.clear()
        self.sign_transition_probs.clear()

        for start in self.sign_nodes:
            queue = deque()

            # best_seen[(node, skipped_count)] = shortest path length seen so far
            # This prevents useless re-expansion of worse versions of the same state.
            best_seen: Dict[Tuple[str, int], int] = {}

            # For each reachable target sign, keep the best candidate:
            # target -> (skipped_count, path)
            best_target_info: Dict[str, Tuple[int, List[str]]] = {}

            # Start from neighbors of the starting sign node
            for nxt in self.adj[start]:
                path = [start, nxt]
                skipped = 0
                queue.append((nxt, path, skipped))
                best_seen[(nxt, skipped)] = len(path)

            while queue:
                cur, path, skipped = queue.popleft()

                # If we reached another sign node, it is a possible next OBSERVED sign
                if cur in sign_node_set and cur != start:
                    prev = best_target_info.get(cur)

                    # Keep better candidate:
                    # 1) fewer skipped signs
                    # 2) if tie, shorter path
                    if (
                        prev is None
                        or skipped < prev[0]
                        or (skipped == prev[0] and len(path) < len(prev[1]))
                    ):
                        best_target_info[cur] = (skipped, path)

                    # Continue exploring beyond this sign as if this sign were skipped
                    if skipped < max_skips:
                        next_skipped = skipped + 1
                        for nxt in self.adj[cur]:
                            next_path = path + [nxt]
                            state = (nxt, next_skipped)

                            if state not in best_seen or len(next_path) < best_seen[state]:
                                best_seen[state] = len(next_path)
                                queue.append((nxt, next_path, next_skipped))
                    continue

                # Ordinary road node: keep exploring with the same skip count
                for nxt in self.adj[cur]:
                    next_path = path + [nxt]
                    state = (nxt, skipped)

                    if state not in best_seen or len(next_path) < best_seen[state]:
                        best_seen[state] = len(next_path)
                        queue.append((nxt, next_path, skipped))

            # Convert best target info into transitions and probabilities
            raw_probs: Dict[str, float] = {}

            for target, (skipped_count, path) in best_target_info.items():
                self.sign_event_edges[start].append(target)
                self.transition_paths[(start, target)] = path
                self.transition_skips[(start, target)] = skipped_count

                raw_probs[target] = (self.p_skip ** skipped_count) * (1.0 - self.p_skip)

            # Normalize outgoing probabilities from this start node
            total = sum(raw_probs.values())
            if total > 0:
                for target, prob in raw_probs.items():
                    self.sign_transition_probs[(start, target)] = prob / total

            # Keep deterministic ordering
            self.sign_event_edges[start] = sorted(
                set(self.sign_event_edges[start]),
                key=lambda x: int(x)
            )

    def observation_prob(self, state: str, observed_sign: str) -> float:
        observed_sign = self._normalize_sign_name(observed_sign)
        node_signs = self.nodes[state].signs
        return self.p_hit if observed_sign in node_signs else self.p_miss

    def transition_prob(self, src: str, dst: str) -> float:
        return self.sign_transition_probs.get((src, dst), 0.0)

    def initial_distribution(self) -> Dict[str, float]:
        print("initial distribution")
        if not self.sign_nodes:
            return {}
        p = 1.0 / len(self.sign_nodes)
        return {s: p for s in self.sign_nodes}

    def forward_filter_init(self, observations: List[str]) -> List[Dict[str, float]]:
        observations = [self._normalize_sign_name(o) for o in observations]
        if not observations:
            return []

        belief = {}
        prior = self.initial_distribution()

        # First observation: correction only
        total = 0.0
        for s in self.sign_nodes:
            belief[s] = prior[s] * self.observation_prob(s, observations[0])
            total += belief[s]
        belief = self._normalize_distribution(belief, total)

        self.beliefs = [belief]
        return belief

    def forward_filter_loop(self, obs: List[str]) -> List[Dict[str, float]]:
        # Remaining observations: predict then correct

        # for obs in observations[1:]:
        predicted = {s: 0.0 for s in self.sign_nodes}
        for src in self.sign_nodes:
            if self.beliefs[-1].get(src, 0.0) == 0.0:
                continue
            for dst in self.sign_event_edges.get(src, []):
                predicted[dst] += self.beliefs[-1][src] * self.transition_prob(src, dst)

        corrected = {}
        total = 0.0
        for s in self.sign_nodes:
            corrected[s] = predicted[s] * self.observation_prob(s, obs)
            total += corrected[s]

        belief = self._normalize_distribution(corrected, total)
        self.beliefs.append(belief)

        return belief

    def viterbi(self, observations: List[str]) -> Tuple[List[str], float]:
        observations = [self._normalize_sign_name(o) for o in observations]
        if not observations:
            return [], 0.0

        states = self.sign_nodes
        if not states:
            return [], 0.0

        # log-space
        tiny = 1e-300
        log = math.log

        dp: List[Dict[str, float]] = []
        parent: List[Dict[str, Optional[str]]] = []

        init = {}
        init_parent = {}
        prior = self.initial_distribution()
        for s in states:
            init[s] = log(max(prior[s], tiny)) + log(max(self.observation_prob(s, observations[0]), tiny))
            init_parent[s] = None
        dp.append(init)
        parent.append(init_parent)

        for t in range(1, len(observations)):
            cur = {}
            cur_parent = {}
            obs = observations[t]

            for dst in states:
                best_score = -float("inf")
                best_prev = None

                for src in states:
                    trans = self.transition_prob(src, dst)
                    if trans <= 0:
                        continue
                    score = dp[t - 1][src] + log(max(trans, tiny)) + log(max(self.observation_prob(dst, obs), tiny))
                    if score > best_score:
                        best_score = score
                        best_prev = src

                cur[dst] = best_score
                cur_parent[dst] = best_prev

            dp.append(cur)
            parent.append(cur_parent)

        last_state = max(dp[-1], key=dp[-1].get)
        best_log_prob = dp[-1][last_state]

        path = [last_state]
        for t in range(len(observations) - 1, 0, -1):
            prev = parent[t][path[-1]]
            if prev is None:
                break
            path.append(prev)
        path.reverse()

        return path, math.exp(best_log_prob) if best_log_prob > -700 else 0.0

    def reconstruct_full_route(self, sign_state_path: List[str]) -> List[str]:
        if not sign_state_path:
            return []
        full = [sign_state_path[0]]
        for a, b in zip(sign_state_path, sign_state_path[1:]):
            segment = self.transition_paths.get((a, b))
            if not segment:
                full.append(b)
            else:
                full.extend(segment[1:])  # avoid duplicating A
        return full

    def explain_route(self, route: List[str]) -> List[dict]:
        explanation = []
        for nid in route:
            info = self.nodes[nid]
            explanation.append(
                {
                    "node": nid,
                    "x": info.x,
                    "y": info.y,
                    "signs": [DISPLAY_NAMES.get(s, s) for s in sorted(info.signs)],
                }
            )
        return explanation

    def top_k_states(self, belief: Dict[str, float], k: int = 10) -> List[Tuple[str, float]]:
        return sorted(belief.items(), key=lambda x: x[1], reverse=True)[:k]

    def _normalize_sign_name(self, sign: str) -> str:
        sign = sign.strip()
        if not sign:
            return sign
        key = sign.lower()
        return SIGN_ALIASES.get(sign, SIGN_ALIASES.get(key, key))

    @staticmethod
    def _normalize_distribution(dist: Dict[str, float], total: float) -> Dict[str, float]:
        if total <= 0:
            return {k: 0.0 for k in dist}
        return {k: v / total for k, v in dist.items()}

    def available_signs(self) -> List[str]:
        return [DISPLAY_NAMES.get(s, s) for s in sorted(self.sign_types)]

    def belief_to_alpha(self, prob, max_prob = 1.00, min_alpha = 0.03, max_alpha = 0.95):

        if prob <= 0 or max_prob <= 0:
            return 0.0

        scaled = prob / max_prob
        scaled = max(0.0, min(1.0, scaled))
        return min_alpha + (max_alpha - min_alpha) * scaled

    def plot_graph_with_beliefs(
        self,
        graphml_path,
        map_path,
        beliefs,
        show_edges=False,
        node_radius=8,
        min_alpha=0.05,
        max_alpha=0.999,
        first_time=0
        ):
        """
        Shows the graph on top of the map.

        Inputs:
        - graphml_path: path to .graphml
        - map_path: path to map image
        - belief_table: python dict like {'30': 0.002, '31': 0.01}

        This function only reads the GraphML file.
        It does NOT save anything into it.
        """
        if first_time:
            self.graph = nx.read_graphml(graphml_path)
            # beliefs = normalize_beliefs(graph, belief_table)

            self.img = Image.open(map_path)
            self.width, self.height = self.img.size

            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(14, 8))
            
        self.ax.clear()
        self.ax.imshow(self.img)
        if show_edges:
            for u, v in self.graph.edges():
                x1 = float(self.graph.nodes[u]['x'])
                y1 = float(self.graph.nodes[u]['y'])
                x2 = float(self.graph.nodes[v]['x'])
                y2 = float(self.graph.nodes[v]['y'])
                self.ax.plot([x1, x2], [y1, y2], linewidth=1.0, alpha=0.25)

        max_prob = max(beliefs.values()) if beliefs else 0.0

        for n, data in self.graph.nodes(data=True):
            x = float(data['x'])
            y = float(data['y'])
            p = beliefs.get(str(n), 0.0)
            alpha = self.belief_to_alpha(p, max_prob, min_alpha, max_alpha)

            circle = Circle(
                (x, y),
                radius=node_radius,
                facecolor=(1.0, 0.0, 0.0, alpha),
                edgecolor='none',
            )
            self.ax.add_patch(circle)
        
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(self.height, 0)
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        plt.tight_layout(pad=0)
        plt.draw()
        plt.pause(0.1)
        

def run_cli() -> None:
    parser = argparse.ArgumentParser(description="Markov localization from sign observations on a GraphML road graph.")
    parser.add_argument("--graph",default = "final_map_graph_v7.graphml", help="Path to the GraphML file.")
    parser.add_argument("--map",default = "map_with_signs.png", help="Path to the PNG file.")

    parser.add_argument("--topk", type=int, default=10, help="How many top states to print per step.")
    parser.add_argument("--json", action="store_true", help="Print structured JSON output.")
    args = parser.parse_args()

    # observations = [x.strip() for x in args.observations.split(",") if x.strip()]
    observations = []
    observations.append(input("Enter the first sign: "))

        # =================================================================================
        #                                       START 
        # =================================================================================

 
    model = # ADD
    model._load_graphml()
    model. # ADD

    beliefs = []
    first_time = 1
    bel = model.# ADD 
    beliefs.append(# ADD)
        for obs in observations:
            print("\n\n\n---------------------\n\n\n")
            if not first_time:
                bel = model.# ADD
                beliefs.append(# ADD)
        
        # =================================================================================
        #                                       END 
        # =================================================================================

        #print(bel)

        best_sign_path, path_prob = model.viterbi(observations)
        full_route = model.reconstruct_full_route(best_sign_path)

        result = {
            "graph": args.graph,
            "observations": observations,
            "available_signs": model.available_signs(),
            "best_sign_node_sequence": best_sign_path,
            "best_route_node_sequence": full_route,
            "best_route_probability": path_prob,
            "best_route_details": model.explain_route(full_route),
            "top_beliefs_per_step": [],
        }

        for i, belief in enumerate(beliefs):
            step = {
                "step": i,
                "observed_sign": observations[i],
                "top_states": [],
            }
            for state, prob in model.top_k_states(belief, k=args.topk):
                info = model.nodes[state]
                step["top_states"].append(
                    {
                        "node": state,
                        "probability": prob,
                        "x": info.x,
                        "y": info.y,
                        "signs": [DISPLAY_NAMES.get(s, s) for s in sorted(info.signs)],
                    }
                )
            result["top_beliefs_per_step"].append(step)

        if args.json:
            print(json.dumps(result, indent=2))
            return

        print("\n=== AVAILABLE SIGN TYPES ===")
        print(", ".join(result["available_signs"]))

        print("\n=== OBSERVATIONS ===")
        print(" -> ".join(observations))

        print("\n=== BEST SIGN NODE SEQUENCE (VITERBI) ===")
        if best_sign_path:
            print(" -> ".join(best_sign_path))
        else:
            print("No valid path found.")

        print("\n=== BEST FULL ROUTE ON ORIGINAL GRAPH ===")
        if full_route:
            print(" -> ".join(full_route))
        else:
            print("No valid route found.")

        print("\n=== TOP BELIEFS AFTER EACH OBSERVATION ===")
        for step in result["top_beliefs_per_step"]:
            print(f"\nStep {step['step']} | observed: {step['observed_sign']}")
            for s in step["top_states"]:
                signs_txt = ",".join(s["signs"]) if s["signs"] else "-"
                #print(
                #    f"  node={s['node']:>4}  p={s['probability']:.4f}  "
                 #   f"xy=({s['x']:.1f},{s['y']:.1f})  signs={signs_txt}"
               # )

        model.plot_graph_with_beliefs(args.graph, args.map, bel, first_time = first_time)
        if first_time:
            first_time = 0
        
        BRIDGE_FILE = "signal.txt"
        print("Waiting for YOLO signal... (Press Ctrl+C in terminal to stop)")
        
        next_sign = None
        while True:
            plt.pause(0.05) # Κρατάει το UI ζωντανό
            
            if os.path.exists(BRIDGE_FILE):
                try:
                    # Μικρή αναμονή για να προλάβει το άλλο script να κλείσει το αρχείο
                    time.sleep(0.05) 
                    with open(BRIDGE_FILE, "r") as f:
                        next_sign = f.read().strip()
                    
                    if next_sign:
                        os.remove(BRIDGE_FILE)
                        print(f"Detected signal: {next_sign}", flush=True)
                        break
                except Exception:
                    pass
            time.sleep(0.1)

        if next_sign == "END":
            break
        observations.append(next_sign) 


if __name__ == "__main__":
    run_cli()

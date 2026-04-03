#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <sstream>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

static constexpr double EPS = 1e-9;

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

struct Point {
  double x = 0.0, y = 0.0;
};

struct AABB {
  double minx = 0.0, maxx = 0.0, miny = 0.0, maxy = 0.0;
};

static AABB bbox2(const Point &p, const Point &q) {
  return {std::min(p.x, q.x), std::max(p.x, q.x), std::min(p.y, q.y),
          std::max(p.y, q.y)};
}

static AABB bbox3(const Point &p, const Point &q, const Point &r) {
  return {std::min(p.x, std::min(q.x, r.x)), std::max(p.x, std::max(q.x, r.x)),
          std::min(p.y, std::min(q.y, r.y)), std::max(p.y, std::max(q.y, r.y))};
}

static AABB mergeBox(const AABB &a, const AABB &b) {
  return {std::min(a.minx, b.minx), std::max(a.maxx, b.maxx),
          std::min(a.miny, b.miny), std::max(a.maxy, b.maxy)};
}

// Node in circular doubly-linked list representing a polygon ring vertex
struct Node {
  Point pt;
  int ring_id = -1;
  int orig_id = -1;
  int prev = -1;
  int next = -1;
  bool alive = false;
  int version = 0; // version counter for lazy staleness detection
  mutable uint64_t edge_query_mark =
      0; // deduplication stamp for spatial queries
};

std::vector<Node> nodes;
std::vector<int> ring_head;
std::vector<int> ring_size;

// ─────────────────────────────────────────────────────────────────────────────
// Geometry utilities
// ─────────────────────────────────────────────────────────────────────────────

inline double cross2(const Point &a, const Point &b, const Point &c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline int sideOf(const Point &p, const Point &a, const Point &b) {
  double cr = cross2(a, b, p);
  return (cr > EPS) ? 1 : (cr < -EPS) ? -1 : 0;
}

inline double distToLine(const Point &p, const Point &a, const Point &b) {
  double dx = b.x - a.x, dy = b.y - a.y;
  double len = std::hypot(dx, dy);
  if (len < EPS)
    return std::hypot(p.x - a.x, p.y - a.y);
  return std::abs((p.x - a.x) * dy - (p.y - a.y) * dx) / len;
}

std::optional<Point> lineLineIntersect(const Point &p1, const Point &p2,
                                       const Point &p3, const Point &p4) {
  double a1 = p2.y - p1.y, b1 = p1.x - p2.x, c1 = a1 * p1.x + b1 * p1.y;
  double a2 = p4.y - p3.y, b2 = p3.x - p4.x, c2 = a2 * p3.x + b2 * p3.y;
  double det = a1 * b2 - a2 * b1;
  if (std::abs(det) < EPS)
    return std::nullopt;
  return Point{(c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det};
}

bool onSegment(const Point &p, const Point &a, const Point &b) {
  return p.x >= std::min(a.x, b.x) - EPS && p.x <= std::max(a.x, b.x) + EPS &&
         p.y >= std::min(a.y, b.y) - EPS && p.y <= std::max(a.y, b.y) + EPS &&
         std::abs(cross2(a, b, p)) < EPS;
}

bool segmentsProperlyIntersect(const Point &p1, const Point &p2,
                               const Point &p3, const Point &p4) {
  double d1 = cross2(p3, p4, p1), d2 = cross2(p3, p4, p2);
  double d3 = cross2(p1, p2, p3), d4 = cross2(p1, p2, p4);
  if (((d1 > EPS && d2 < -EPS) || (d1 < -EPS && d2 > EPS)) &&
      ((d3 > EPS && d4 < -EPS) || (d3 < -EPS && d4 > EPS)))
    return true;
  if (std::abs(d1) < EPS && onSegment(p1, p3, p4))
    return true;
  if (std::abs(d2) < EPS && onSegment(p2, p3, p4))
    return true;
  if (std::abs(d3) < EPS && onSegment(p3, p1, p2))
    return true;
  if (std::abs(d4) < EPS && onSegment(p4, p1, p2))
    return true;
  return false;
}

bool pointInTriangle(const Point &p, const Point &a, const Point &b,
                     const Point &c) {
  double s1 = cross2(a, b, p);
  double s2 = cross2(b, c, p);
  double s3 = cross2(c, a, p);
  double abc = cross2(a, b, c);
  if (std::abs(abc) <= EPS)
    return false;
  if (std::abs(s1) <= EPS || std::abs(s2) <= EPS || std::abs(s3) <= EPS)
    return false;
  bool allPos = (s1 > 0.0 && s2 > 0.0 && s3 > 0.0);
  bool allNeg = (s1 < 0.0 && s2 < 0.0 && s3 < 0.0);
  if (abc > 0.0)
    return allPos;
  return allNeg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Steiner Point Computation (Kronenfeld et al. 2020)
// ─────────────────────────────────────────────────────────────────────────────

bool computeSteinerPoint(const Point &A, const Point &B, const Point &C,
                         const Point &D, Point &outE) {
  if (std::hypot(D.x - A.x, D.y - A.y) < EPS) {
    outE = A;
    return true;
  }
  double a = D.y - A.y;
  double b = A.x - D.x;
  double c_line =
      -B.y * A.x + (A.y - C.y) * B.x + (B.y - D.y) * C.x + C.y * D.x;

  double dA = std::abs(a * A.x + b * A.y + c_line);
  double dD = std::abs(a * D.x + b * D.y + c_line);
  if (dA < EPS && dD < EPS) {
    outE = {(A.x + D.x) * 0.5, (A.y + D.y) * 0.5};
    return true;
  }

  Point el0, el1;
  if (std::abs(b) > 1e-13) {
    el0 = {0.0, -c_line / b};
    el1 = {1.0, -(a + c_line) / b};
  } else if (std::abs(a) > 1e-13) {
    el0 = {-c_line / a, 0.0};
    el1 = {-c_line / a, 1.0};
  } else {
    return false;
  }

  int sideB_AD = sideOf(B, A, D);
  int sideC_AD = sideOf(C, A, D);
  bool useAB;
  if (sideB_AD == sideC_AD) {
    useAB = (distToLine(B, A, D) >= distToLine(C, A, D) - EPS);
  } else {
    int sideEline_AD = sideOf(el0, A, D);
    if (sideEline_AD == 0)
      sideEline_AD = sideOf(el1, A, D);
    useAB = (sideEline_AD == sideB_AD);
  }

  auto optE = lineLineIntersect(useAB ? A : C, useAB ? B : D, el0, el1);
  if (!optE)
    return false;
  outE = *optE;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Areal Displacement Calculation
// ─────────────────────────────────────────────────────────────────────────────

double arealDisplacement(const Point &A, const Point &B, const Point &C,
                         const Point &D, const Point &E) {
  auto tri = [](const Point &p, const Point &q, const Point &r) {
    return 0.5 *
           std::abs((q.x - p.x) * (r.y - p.y) - (r.x - p.x) * (q.y - p.y));
  };
  Point P1 = B;
  auto opt1 = lineLineIntersect(A, E, B, C);
  if (opt1 && onSegment(*opt1, A, E) && onSegment(*opt1, B, C))
    P1 = *opt1;

  Point P2 = C;
  auto opt2 = lineLineIntersect(E, D, B, C);
  if (opt2 && onSegment(*opt2, E, D) && onSegment(*opt2, B, C))
    P2 = *opt2;

  return tri(A, B, P1) + tri(P1, E, P2) + tri(P2, C, D);
}

// ─────────────────────────────────────────────────────────────────────────────
// Candidate structure for lazy priority queue
// ─────────────────────────────────────────────────────────────────────────────

struct Candidate {
  int A, B, C, D;             // node indices of the 4-vertex window
  Point E;                    // computed Steiner point
  double disp;                // areal displacement cost
  int verA, verB, verC, verD; // version stamps at creation time

  bool isStale() const {
    return !nodes[A].alive || !nodes[B].alive || !nodes[C].alive ||
           !nodes[D].alive || nodes[A].version != verA ||
           nodes[B].version != verB || nodes[C].version != verC ||
           nodes[D].version != verD;
  }

  // Min-heap: lower displacement is better (use greater-than for
  // std::priority_queue)
  bool operator>(const Candidate &o) const {
    if (std::abs(disp - o.disp) > 1e-12)
      return disp > o.disp;
    // Tie-breaking: prefer specific ring/vertex ordering for determinism
    if (nodes[B].ring_id != nodes[o.B].ring_id)
      return nodes[B].ring_id > nodes[o.B].ring_id;
    if (nodes[B].orig_id != nodes[o.B].orig_id)
      return nodes[B].orig_id > nodes[o.B].orig_id;
    if (nodes[C].orig_id != nodes[o.C].orig_id)
      return nodes[C].orig_id > nodes[o.C].orig_id;
    if (nodes[A].orig_id != nodes[o.A].orig_id)
      return nodes[A].orig_id > nodes[o.A].orig_id;
    if (nodes[D].orig_id != nodes[o.D].orig_id)
      return nodes[D].orig_id > nodes[o.D].orig_id;
    if (std::abs(E.x - o.E.x) > 1e-12)
      return E.x > o.E.x;
    return E.y > o.E.y;
  }
};

using PQ = std::priority_queue<Candidate, std::vector<Candidate>,
                               std::greater<Candidate>>;

// ─────────────────────────────────────────────────────────────────────────────
// Spatial Hash Index
// ─────────────────────────────────────────────────────────────────────────────

struct SpatialHash {
  double minx, miny, cell;
  int table_size;
  std::vector<std::vector<int>> point_cells;
  std::vector<std::vector<int>> edge_cells;
  mutable uint64_t query_stamp = 0;

  void init(int totalVerts, double mx, double my, double w, double h) {
    table_size = std::max(1024, totalVerts * 2);
    point_cells.resize(table_size);
    edge_cells.resize(table_size);
    minx = mx;
    miny = my;
    double maxDim = std::max(w, h);
    double gridN = std::max(64.0, std::sqrt((double)std::max(1, totalVerts)));
    cell = std::max(maxDim / gridN, 1e-9);
  }

  inline size_t hash_cell(int cx, int cy) const {
    size_t h = (size_t)cx * 73856093 ^ (size_t)cy * 19349663;
    return h % table_size;
  }

  int ix(double x) const { return (int)std::floor((x - minx) / cell); }
  int iy(double y) const { return (int)std::floor((y - miny) / cell); }

  void insert_point(const Point &pt, int id) {
    size_t h = hash_cell(ix(pt.x), iy(pt.y));
    point_cells[h].push_back(id);
  }

  void remove_point(const Point &pt, int id) {
    size_t h = hash_cell(ix(pt.x), iy(pt.y));
    auto &vec = point_cells[h];
    auto it = std::find(vec.begin(), vec.end(), id);
    if (it != vec.end()) {
      *it = vec.back();
      vec.pop_back();
    }
  }

  void insert_edge(const Point &a, const Point &b, int id) {
    int x0 = ix(std::min(a.x, b.x) - 1e-9);
    int x1 = ix(std::max(a.x, b.x) + 1e-9);
    int y0 = iy(std::min(a.y, b.y) - 1e-9);
    int y1 = iy(std::max(a.y, b.y) + 1e-9);
    for (int x = x0; x <= x1; ++x) {
      for (int y = y0; y <= y1; ++y) {
        edge_cells[hash_cell(x, y)].push_back(id);
      }
    }
  }

  void remove_edge(const Point &a, const Point &b, int id) {
    int x0 = ix(std::min(a.x, b.x) - 1e-9);
    int x1 = ix(std::max(a.x, b.x) + 1e-9);
    int y0 = iy(std::min(a.y, b.y) - 1e-9);
    int y1 = iy(std::max(a.y, b.y) + 1e-9);
    for (int x = x0; x <= x1; ++x) {
      for (int y = y0; y <= y1; ++y) {
        auto &vec = edge_cells[hash_cell(x, y)];
        auto it = std::find(vec.begin(), vec.end(), id);
        if (it != vec.end()) {
          *it = vec.back();
          vec.pop_back();
        }
      }
    }
  }

  void query_edge_starts(const AABB &box, std::vector<int> &res) const {
    int x0 = ix(box.minx - 1e-9);
    int x1 = ix(box.maxx + 1e-9);
    int y0 = iy(box.miny - 1e-9);
    int y1 = iy(box.maxy + 1e-9);
    ++query_stamp;
    if (query_stamp == 0)
      query_stamp = 1;
    res.clear();
    for (int x = x0; x <= x1; ++x) {
      for (int y = y0; y <= y1; ++y) {
        size_t k = hash_cell(x, y);
        for (int id : edge_cells[k]) {
          if (nodes[id].edge_query_mark != query_stamp) {
            nodes[id].edge_query_mark = query_stamp;
            res.push_back(id);
          }
        }
      }
    }
  }

  void query_points(const AABB &box, std::vector<int> &res) const {
    int x0 = ix(box.minx - 1e-9);
    int x1 = ix(box.maxx + 1e-9);
    int y0 = iy(box.miny - 1e-9);
    int y1 = iy(box.maxy + 1e-9);
    res.clear();
    for (int x = x0; x <= x1; ++x) {
      for (int y = y0; y <= y1; ++y) {
        size_t k = hash_cell(x, y);
        for (int id : point_cells[k]) {
          res.push_back(id);
        }
      }
    }
  }
} spatial;

// ─────────────────────────────────────────────────────────────────────────────
// Topology Safety Check
// ─────────────────────────────────────────────────────────────────────────────

bool collapseIsTopologicallySafe(int A_id, int B_id, int C_id, int D_id,
                                 const Point &E) {
  const Point &A = nodes[A_id].pt;
  const Point &B = nodes[B_id].pt;
  const Point &C = nodes[C_id].pt;
  const Point &D = nodes[D_id].pt;

  // Check new edges A-E and E-D against all nearby edges
  AABB segBox = mergeBox(bbox2(A, E), bbox2(E, D));
  std::vector<int> edges;
  spatial.query_edge_starts(segBox, edges);
  for (int u_id : edges) {
    if (!nodes[u_id].alive)
      continue;
    int v_id = nodes[u_id].next;
    if (!nodes[v_id].alive)
      continue;
    const Point &u = nodes[u_id].pt;
    const Point &v = nodes[v_id].pt;

    // Skip edges adjacent to the collapse window
    bool isAdjacent =
        (u_id == A_id || u_id == B_id || u_id == C_id || u_id == D_id ||
         v_id == A_id || v_id == B_id || v_id == C_id || v_id == D_id);
    if (isAdjacent)
      continue;

    if (segmentsProperlyIntersect(A, E, u, v))
      return false;
    if (segmentsProperlyIntersect(E, D, u, v))
      return false;
  }

  // Check no vertex falls inside the swept triangles
  auto checkTri = [&](const Point &p1, const Point &p2, const Point &p3) {
    AABB triBox = bbox3(p1, p2, p3);
    std::vector<int> pts;
    spatial.query_points(triBox, pts);
    for (int p_id : pts) {
      if (!nodes[p_id].alive)
        continue;
      if (p_id == A_id || p_id == B_id || p_id == C_id || p_id == D_id)
        continue;
      if (pointInTriangle(nodes[p_id].pt, p1, p2, p3))
        return false;
    }
    return true;
  };

  if (!checkTri(A, B, E))
    return false;
  if (!checkTri(E, B, C))
    return false;
  if (!checkTri(E, C, D))
    return false;

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Candidate Creation
// ─────────────────────────────────────────────────────────────────────────────

void pushCandidate(int A_id, int B_id, int C_id, int D_id, PQ &pq) {
  if (A_id == C_id || B_id == D_id)
    return;
  if (!nodes[A_id].alive || !nodes[B_id].alive || !nodes[C_id].alive ||
      !nodes[D_id].alive)
    return;

  Point E;
  if (!computeSteinerPoint(nodes[A_id].pt, nodes[B_id].pt, nodes[C_id].pt,
                           nodes[D_id].pt, E))
    return;
  if (!std::isfinite(E.x) || !std::isfinite(E.y))
    return;

  double disp = arealDisplacement(nodes[A_id].pt, nodes[B_id].pt,
                                  nodes[C_id].pt, nodes[D_id].pt, E);
  if (!std::isfinite(disp))
    return;

  Candidate c;
  c.A = A_id;
  c.B = B_id;
  c.C = C_id;
  c.D = D_id;
  c.E = E;
  c.disp = disp;
  c.verA = nodes[A_id].version;
  c.verB = nodes[B_id].version;
  c.verC = nodes[C_id].version;
  c.verD = nodes[D_id].version;
  pq.push(c);
}

// Push candidates in the neighborhood of a vertex (spanning ~8 windows)
void pushNeighborCandidates(int E_id, PQ &pq, int rSize) {
  if (rSize < 5)
    return;
  // Walk 4 predecessors back
  int start = E_id;
  for (int i = 0; i < 4; ++i)
    start = nodes[start].prev;
  int span = std::min(rSize, 8);
  for (int i = 0; i < span; ++i) {
    int a = start;
    int b = nodes[a].next;
    int c = nodes[b].next;
    int d = nodes[c].next;
    pushCandidate(a, b, c, d, pq);
    start = nodes[start].next;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Lookahead: estimate best future displacement after a hypothetical collapse
// ─────────────────────────────────────────────────────────────────────────────

static double localCandidateDisp(const Point &A, const Point &B, const Point &C,
                                 const Point &D) {
  Point E;
  if (!computeSteinerPoint(A, B, C, D, E) || !std::isfinite(E.x) ||
      !std::isfinite(E.y))
    return std::numeric_limits<double>::infinity();
  double d = arealDisplacement(A, B, C, D, E);
  if (!std::isfinite(d))
    return std::numeric_limits<double>::infinity();
  return d;
}

static double estimateNextDispAfterCollapse(const Candidate &cand) {
  int P_id = nodes[cand.A].prev;
  int N_id = nodes[cand.D].next;
  const Point &A = nodes[cand.A].pt;
  const Point &D = nodes[cand.D].pt;
  const Point &E = cand.E;

  double best = std::numeric_limits<double>::infinity();

  // Check left window: P -> A -> E -> D
  if (P_id != cand.B && P_id != cand.C && P_id != cand.D) {
    best = std::min(best, localCandidateDisp(nodes[P_id].pt, A, E, D));
  }
  // Check right window: A -> E -> D -> N
  if (N_id != cand.A && N_id != cand.B && N_id != cand.C) {
    best = std::min(best, localCandidateDisp(A, E, D, nodes[N_id].pt));
  }

  if (!std::isfinite(best))
    return 0.0;
  return best;
}

// ─────────────────────────────────────────────────────────────────────────────
// Area computation
// ─────────────────────────────────────────────────────────────────────────────

double signedArea(int ring_id) {
  if (ring_size[ring_id] < 3)
    return 0.0;
  double area = 0.0;
  int start = ring_head[ring_id];
  int curr = start;
  do {
    int nxt = nodes[curr].next;
    area += (nodes[curr].pt.x * nodes[nxt].pt.y) -
            (nodes[nxt].pt.x * nodes[curr].pt.y);
    curr = nxt;
  } while (curr != start);
  return area * 0.5;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: ./simplify <input.csv> <target_vertices>\n";
    return 1;
  }
  int target = std::stoi(argv[2]);

  // ── Parse CSV input ──
  std::ifstream in(argv[1]);
  if (!in) {
    std::cerr << "Cannot open " << argv[1] << "\n";
    return 1;
  }
  std::string line;
  std::getline(in, line); // skip header

  std::map<int, std::vector<std::pair<int, Point>>> raw_rings;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty())
      continue;
    std::istringstream ss(line);
    std::string t;
    int rid, vid;
    Point p;
    if (!std::getline(ss, t, ','))
      continue;
    rid = std::stoi(t);
    if (!std::getline(ss, t, ','))
      continue;
    vid = std::stoi(t);
    if (!std::getline(ss, t, ','))
      continue;
    p.x = std::stod(t);
    if (!std::getline(ss, t, ','))
      continue;
    p.y = std::stod(t);
    raw_rings[rid].push_back({vid, p});
  }

  int totalVerts = 0;
  for (auto &pair : raw_rings) {
    std::sort(pair.second.begin(), pair.second.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    totalVerts += pair.second.size();
  }

  // ── Build circular linked lists ──
  nodes.reserve(totalVerts);
  ring_head.resize(raw_rings.size());
  ring_size.resize(raw_rings.size());

  int ri = 0;
  for (auto &pair : raw_rings) {
    auto &entries = pair.second;
    int r_start = nodes.size();
    ring_head[ri] = r_start;
    ring_size[ri] = entries.size();
    for (int i = 0; i < (int)entries.size(); ++i) {
      Node n;
      n.pt = entries[i].second;
      n.ring_id = ri;
      n.orig_id = entries[i].first;
      n.alive = true;
      n.version = 0;
      n.prev = r_start + (i - 1 + entries.size()) % entries.size();
      n.next = r_start + (i + 1) % entries.size();
      nodes.push_back(n);
    }
    ri++;
  }

  // ── Compute input area ──
  double area_in = 0.0;
  for (int i = 0; i < (int)ring_head.size(); ++i)
    area_in += signedArea(i);

  // ── Read lookahead weight from environment ──
  double lookaheadW = 0.35;
  if (const char *env_w = std::getenv("SIMPLIFY_LOOKAHEAD_W"))
    lookaheadW = std::stod(env_w);

  int lookaheadK = 0;
  if (const char *env_k = std::getenv("SIMPLIFY_LOOKAHEAD_K")) {
    lookaheadK = std::stoi(env_k);
    if (lookaheadK < 0)
      lookaheadK = 0;
    if (lookaheadK > 32)
      lookaheadK = 32;
  }
  int selectionK = std::max(1, lookaheadK);
  int scanBudget = std::max(selectionK, selectionK * 4);

  double totalDisp = 0.0;

  if (totalVerts > target) {
    // ── Build spatial index ──
    double minx = 1e30, maxx = -1e30;
    double miny = 1e30, maxy = -1e30;
    for (const auto &node : nodes) {
      minx = std::min(minx, node.pt.x);
      maxx = std::max(maxx, node.pt.x);
      miny = std::min(miny, node.pt.y);
      maxy = std::max(maxy, node.pt.y);
    }

    spatial.init(totalVerts, minx, miny, std::max(maxx - minx, 1e-9),
                 std::max(maxy - miny, 1e-9));

    for (int i = 0; i < totalVerts; ++i) {
      spatial.insert_point(nodes[i].pt, i);
      spatial.insert_edge(nodes[i].pt, nodes[nodes[i].next].pt, i);
    }

    // ── Seed priority queue with all valid 4-vertex windows ──
    PQ pq;
    for (int i = 0; i < totalVerts; ++i) {
      if (ring_size[nodes[i].ring_id] < 5)
        continue;
      int a = i;
      int b = nodes[a].next;
      int c = nodes[b].next;
      int d = nodes[c].next;
      pushCandidate(a, b, c, d, pq);
    }

    // ── Greedy collapse loop ──
    long long collapseCount = 0;

    while (totalVerts > target && !pq.empty()) {
      // With lookahead: gather multiple feasible candidates and pick the best
      std::vector<Candidate> feasible;
      int scanned = 0;

      while (!pq.empty() && scanned < scanBudget &&
             (int)feasible.size() < selectionK) {
        Candidate cand = pq.top();
        pq.pop();
        ++scanned;

        if (cand.isStale())
          continue;

        int r = nodes[cand.B].ring_id;
        if (ring_size[r] <= 4)
          continue;

        if (!collapseIsTopologicallySafe(cand.A, cand.B, cand.C, cand.D,
                                         cand.E))
          continue;

        feasible.push_back(cand);
      }

      if (feasible.empty())
        continue;

      // Pick the best candidate (with lookahead scoring if K > 1)
      int pick = 0;
      if (selectionK > 1 && feasible.size() > 1) {
        double bestScore = std::numeric_limits<double>::infinity();
        for (int i = 0; i < (int)feasible.size(); ++i) {
          const Candidate &c = feasible[i];
          double score = c.disp + lookaheadW * estimateNextDispAfterCollapse(c);
          if (score + 1e-12 < bestScore) {
            bestScore = score;
            pick = i;
          } else if (std::abs(score - bestScore) <= 1e-12 &&
                     c.disp + 1e-12 < feasible[pick].disp) {
            pick = i;
          }
        }
      }

      Candidate cand = feasible[pick];
      // Push back non-selected feasible candidates
      for (int i = 0; i < (int)feasible.size(); ++i) {
        if (i != pick)
          pq.push(feasible[i]);
      }

      int A_id = cand.A;
      int B_id = cand.B;
      int C_id = cand.C;
      int D_id = cand.D;
      Point E = cand.E;

      // Double-check staleness after potential re-ordering
      if (cand.isStale() || ring_size[nodes[B_id].ring_id] <= 4)
        continue;

      // ── Apply the collapse ──
      Point old_B = nodes[B_id].pt;
      Point old_C = nodes[C_id].pt;

      // Update spatial index: remove old geometry
      spatial.remove_point(old_C, C_id);
      spatial.remove_point(old_B, B_id);
      spatial.remove_edge(nodes[A_id].pt, old_B, A_id);
      spatial.remove_edge(old_B, old_C, B_id);
      spatial.remove_edge(old_C, nodes[D_id].pt, C_id);

      // Move B to Steiner point, kill C
      nodes[B_id].pt = E;
      nodes[B_id].version++;
      nodes[C_id].alive = false;
      nodes[C_id].version++;

      // Rewire linked list
      nodes[A_id].next = B_id;
      nodes[B_id].prev = A_id;
      nodes[B_id].next = D_id;
      nodes[D_id].prev = B_id;

      // Bump neighbor versions to invalidate stale candidates
      nodes[A_id].version++;
      nodes[D_id].version++;

      ring_size[nodes[B_id].ring_id]--;
      if (ring_head[nodes[B_id].ring_id] == C_id)
        ring_head[nodes[B_id].ring_id] = B_id;

      totalVerts--;
      totalDisp += cand.disp;
      collapseCount++;

      // Update spatial index: insert new geometry
      spatial.insert_point(E, B_id);
      spatial.insert_edge(nodes[A_id].pt, E, A_id);
      spatial.insert_edge(E, nodes[D_id].pt, B_id);

      // Push neighbor candidates
      pushNeighborCandidates(B_id, pq, ring_size[nodes[B_id].ring_id]);
    }
  }

  // ── Compute output area ──
  double area_out = 0.0;
  for (int i = 0; i < (int)ring_head.size(); ++i)
    area_out += signedArea(i);

  // ── Print output ──
  std::cout << "ring_id,vertex_id,x,y\n";
  for (int ri = 0; ri < (int)ring_head.size(); ++ri) {
    if (ring_size[ri] == 0)
      continue;
    int start = ring_head[ri];

    // Find canonical head (smallest original vertex ID)
    int best = start;
    int curr = nodes[start].next;
    while (curr != start) {
      if (nodes[curr].orig_id < nodes[best].orig_id)
        best = curr;
      curr = nodes[curr].next;
    }

    curr = best;
    int vid = 0;
    do {
      std::cout << ri << "," << vid++ << "," << std::defaultfloat
                << std::setprecision(10) << nodes[curr].pt.x << ","
                << nodes[curr].pt.y << "\n";
      curr = nodes[curr].next;
    } while (curr != best);
  }

  std::cout << std::scientific << std::setprecision(6)
            << "Total signed area in input: " << area_in << "\n"
            << "Total signed area in output: " << area_out << "\n"
            << "Total areal displacement: " << totalDisp << "\n";

  return 0;
}

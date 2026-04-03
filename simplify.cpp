#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

constexpr double kEps = 1e-9;

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Ring {
    int id = 0;
    std::vector<Point> vertices;
};

struct Polygon {
    std::vector<Ring> rings;
};

struct TopologyConstraints {
    std::vector<std::vector<bool>> contains;
};

struct Candidate {
    bool valid = false;
    std::size_t ringIndex = 0;
    std::size_t vertexIndex = 0;
    bool moveNext = true;
    Point replacement;
    double displacement = 0.0;
};

double cross(const Point& a, const Point& b) {
    return a.x * b.y - a.y * b.x;
}

double cross(const Point& a, const Point& b, const Point& c) {
    return cross({b.x - a.x, b.y - a.y}, {c.x - a.x, c.y - a.y});
}

Point operator+(const Point& a, const Point& b) {
    return {a.x + b.x, a.y + b.y};
}

Point operator-(const Point& a, const Point& b) {
    return {a.x - b.x, a.y - b.y};
}

Point operator*(const Point& a, double t) {
    return {a.x * t, a.y * t};
}

double dot(const Point& a, const Point& b) {
    return a.x * b.x + a.y * b.y;
}

double norm2(const Point& a) {
    return dot(a, a);
}

double signedArea2(const std::vector<Point>& vertices) {
    double total = 0.0;
    const std::size_t n = vertices.size();
    for (std::size_t i = 0; i < n; ++i) {
        total += cross(vertices[i], vertices[(i + 1) % n]);
    }
    return total;
}

double signedArea(const std::vector<Point>& vertices) {
    return 0.5 * signedArea2(vertices);
}

bool nearlyEqual(double a, double b, double eps = kEps) {
    return std::abs(a - b) <= eps;
}

int orientation(const Point& a, const Point& b, const Point& c) {
    const double value = cross(a, b, c);
    if (value > kEps) {
        return 1;
    }
    if (value < -kEps) {
        return -1;
    }
    return 0;
}

bool onSegment(const Point& a, const Point& b, const Point& p) {
    return std::min(a.x, b.x) - kEps <= p.x && p.x <= std::max(a.x, b.x) + kEps &&
           std::min(a.y, b.y) - kEps <= p.y && p.y <= std::max(a.y, b.y) + kEps &&
           std::abs(cross(a, b, p)) <= kEps;
}

bool segmentsIntersect(const Point& a, const Point& b, const Point& c, const Point& d) {
    const int o1 = orientation(a, b, c);
    const int o2 = orientation(a, b, d);
    const int o3 = orientation(c, d, a);
    const int o4 = orientation(c, d, b);

    if (o1 != o2 && o3 != o4) {
        return true;
    }
    if (o1 == 0 && onSegment(a, b, c)) {
        return true;
    }
    if (o2 == 0 && onSegment(a, b, d)) {
        return true;
    }
    if (o3 == 0 && onSegment(c, d, a)) {
        return true;
    }
    if (o4 == 0 && onSegment(c, d, b)) {
        return true;
    }
    return false;
}

bool pointInRing(const std::vector<Point>& ring, const Point& p) {
    bool inside = false;
    const std::size_t n = ring.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point& a = ring[j];
        const Point& b = ring[i];
        if (onSegment(a, b, p)) {
            return true;
        }
        const bool crosses = ((a.y > p.y) != (b.y > p.y));
        if (crosses) {
            const double x = a.x + (b.x - a.x) * (p.y - a.y) / (b.y - a.y);
            if (x >= p.x - kEps) {
                inside = !inside;
            }
        }
    }
    return inside;
}

bool ringSimple(const std::vector<Point>& ring) {
    const std::size_t n = ring.size();
    if (n < 3) {
        return false;
    }
    for (std::size_t i = 0; i < n; ++i) {
        Point a = ring[i];
        Point b = ring[(i + 1) % n];
        for (std::size_t j = i + 1; j < n; ++j) {
            const std::size_t iNext = (i + 1) % n;
            const std::size_t jNext = (j + 1) % n;
            if (i == j || i == jNext || iNext == j || iNext == jNext) {
                continue;
            }
            Point c = ring[j];
            Point d = ring[jNext];
            if (segmentsIntersect(a, b, c, d)) {
                return false;
            }
        }
    }
    return true;
}

bool ringsIntersect(const std::vector<Point>& a, const std::vector<Point>& b) {
    for (std::size_t i = 0; i < a.size(); ++i) {
        Point a0 = a[i];
        Point a1 = a[(i + 1) % a.size()];
        for (std::size_t j = 0; j < b.size(); ++j) {
            Point b0 = b[j];
            Point b1 = b[(j + 1) % b.size()];
            if (segmentsIntersect(a0, a1, b0, b1)) {
                return true;
            }
        }
    }
    return false;
}

std::vector<Point> applyCandidateToRing(const Ring& ring, std::size_t vertexIndex, bool moveNext, const Point& replacement) {
    std::vector<Point> result = ring.vertices;
    const std::size_t n = result.size();
    if (moveNext) {
        result[(vertexIndex + 1) % n] = replacement;
    } else {
        result[(vertexIndex + n - 1) % n] = replacement;
    }
    result.erase(result.begin() + static_cast<std::ptrdiff_t>(vertexIndex));
    return result;
}

TopologyConstraints buildTopologyConstraints(const Polygon& polygon) {
    TopologyConstraints constraints;
    const std::size_t n = polygon.rings.size();
    constraints.contains.assign(n, std::vector<bool>(n, false));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) {
                continue;
            }
            constraints.contains[i][j] = pointInRing(polygon.rings[i].vertices, polygon.rings[j].vertices.front());
        }
    }
    return constraints;
}

bool topologyPreserved(const Polygon& polygon,
                       const TopologyConstraints& constraints,
                       std::size_t ringIndex,
                       const std::vector<Point>& candidateRing) {
    if (!ringSimple(candidateRing)) {
        return false;
    }

    for (std::size_t i = 0; i < polygon.rings.size(); ++i) {
        if (i == ringIndex) {
            continue;
        }
        const auto& other = polygon.rings[i].vertices;
        if (ringsIntersect(candidateRing, other)) {
            return false;
        }
        if (pointInRing(candidateRing, other.front()) != constraints.contains[ringIndex][i]) {
            return false;
        }
        if (pointInRing(other, candidateRing.front()) != constraints.contains[i][ringIndex]) {
            return false;
        }
    }
    return true;
}

Candidate buildCandidate(const Polygon& polygon,
                         const TopologyConstraints& constraints,
                         std::size_t ringIndex,
                         std::size_t vertexIndex,
                         bool moveNext) {
    Candidate best;
    const Ring& ring = polygon.rings[ringIndex];
    const auto& v = ring.vertices;
    const std::size_t n = v.size();
    if (n <= 3) {
        return best;
    }

    std::vector<Point> updatedRing = ring.vertices;
    updatedRing.erase(updatedRing.begin() + static_cast<std::ptrdiff_t>(vertexIndex));
    if (updatedRing.size() < 3) {
        return best;
    }

    const double missingArea2 = signedArea2(v) - signedArea2(updatedRing);
    const std::size_t adjustedIndex = moveNext
                                          ? (vertexIndex % updatedRing.size())
                                          : ((vertexIndex + updatedRing.size() - 1) % updatedRing.size());
    const std::size_t adjustedPrev = (adjustedIndex + updatedRing.size() - 1) % updatedRing.size();
    const std::size_t adjustedNext = (adjustedIndex + 1) % updatedRing.size();
    const Point baseline = updatedRing[adjustedPrev] - updatedRing[adjustedNext];
    const Point normal = {-baseline.y, baseline.x};
    const double denom = norm2(normal);
    if (denom <= kEps) {
        return best;
    }

    Point originalPoint = updatedRing[adjustedIndex];

    const double scale = missingArea2 / denom;
    const Point replacement = originalPoint + normal * scale;

    // Reject if replacement moves too far
    const double maxMove = 300.0;
    if (std::sqrt(norm2(replacement - originalPoint)) > maxMove) {
        return best;
    }

    updatedRing[adjustedIndex] = replacement;

    if (!nearlyEqual(signedArea2(updatedRing), signedArea2(v), 1e-3)) {
        return best;
    }
    if (!topologyPreserved(polygon, constraints, ringIndex, updatedRing)) {
        return best;
    }

    best.valid = true;
    best.ringIndex = ringIndex;
    best.vertexIndex = vertexIndex;
    best.moveNext = moveNext;
    best.replacement = replacement;
    //const Point adjustedOriginal = moveNext ? v[(vertexIndex + 1) % n] : v[(vertexIndex + n - 1) % n];
    double areaCost = std::abs(0.5 * missingArea2);

    // small penalty for large movement (helps shape stability)
    double moveCost = std::sqrt(norm2(replacement - originalPoint));

    best.displacement = areaCost + 0.01 * moveCost;
    return best;
}

Polygon readPolygonCsv(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Unable to open input file: " + path);
    }

    std::string line;
    std::getline(input, line);

    std::map<int, std::vector<std::pair<int, Point>>> grouped;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> parts;
        while (std::getline(ss, token, ',')) {
            parts.push_back(token);
        }
        if (parts.size() != 4) {
            throw std::runtime_error("Malformed CSV row: " + line);
        }
        const int ringId = std::stoi(parts[0]);
        const int vertexId = std::stoi(parts[1]);
        const double x = std::stod(parts[2]);
        const double y = std::stod(parts[3]);
        grouped[ringId].push_back({vertexId, {x, y}});
    }

    Polygon polygon;
    for (auto& [ringId, entries] : grouped) {
        std::sort(entries.begin(), entries.end(),
                  [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        Ring ring;
        ring.id = ringId;
        for (const auto& [_, point] : entries) {
            ring.vertices.push_back(point);
        }
        polygon.rings.push_back(std::move(ring));
    }
    return polygon;
}

std::size_t totalVertices(const Polygon& polygon) {
    std::size_t count = 0;
    for (const auto& ring : polygon.rings) {
        count += ring.vertices.size();
    }
    return count;
}

double totalSignedArea(const Polygon& polygon) {
    double total = 0.0;
    for (const auto& ring : polygon.rings) {
        total += signedArea(ring.vertices);
    }
    return total;
}

double simplifyPolygon(Polygon& polygon, std::size_t targetVertices) {
    const TopologyConstraints constraints = buildTopologyConstraints(polygon);
    double totalDisplacement = 0.0;
    while (totalVertices(polygon) > targetVertices) {
        //std::cout << "Vertices left: " << totalVertices(polygon) << std::endl;
        Candidate best;
        double bestCost = std::numeric_limits<double>::infinity();

        for (std::size_t ringIndex = 0; ringIndex < polygon.rings.size(); ++ringIndex) {
            const auto& ring = polygon.rings[ringIndex].vertices;
            const std::size_t MIN_RING_SIZE = (ringIndex == 0) ? 8 : 4;

            if (ring.size() <= MIN_RING_SIZE) {
                continue;
            }
            for (std::size_t vertexIndex = 0; vertexIndex < ring.size(); ++vertexIndex) {
                for (bool moveNext : {true, false}) {
                    Candidate candidate = buildCandidate(polygon, constraints, ringIndex, vertexIndex, moveNext);
                    if (!candidate.valid) continue;

                    double adjustedCost = candidate.displacement;

                    // discourage collapsing outer ring slightly
                    if (candidate.ringIndex == 0) {
                        adjustedCost *= 1.2;
                    }

                    if (adjustedCost < bestCost) {
                        best = candidate;
                        bestCost = adjustedCost;
                    }
                }
            }
        }

        if (!best.valid) {
            break;
        }

        Ring& ring = polygon.rings[best.ringIndex];
        ring.vertices = applyCandidateToRing(ring, best.vertexIndex, best.moveNext, best.replacement);
        totalDisplacement += best.displacement;
    }
    return totalDisplacement;
}

void printPolygon(const Polygon& polygon, double inputArea, double outputArea, double displacement) {
    std::cout << "ring_id,vertex_id,x,y\n";
    std::cout << std::setprecision(10);
    for (const auto& ring : polygon.rings) {
        for (std::size_t i = 0; i < ring.vertices.size(); ++i) {
            const auto& p = ring.vertices[i];
            std::cout << ring.id << "," << i << "," << p.x << "," << p.y << "\n";
        }
    }
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "Total signed area in input: " << inputArea << "\n";
    std::cout << "Total signed area in output: " << outputArea << "\n";
    std::cout << "Total areal displacement: " << displacement << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 3) {
            std::cerr << "Usage: " << argv[0] << " <input_file> <target_vertices>\n";
            return 1;
        }

        Polygon polygon = readPolygonCsv(argv[1]);
        const std::size_t targetVertices = static_cast<std::size_t>(std::stoul(argv[2]));
        const double inputArea = totalSignedArea(polygon);
        const double displacement = simplifyPolygon(polygon, targetVertices);
        const double outputArea = totalSignedArea(polygon);
        printPolygon(polygon, inputArea, outputArea, displacement);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
}

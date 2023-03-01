#include <array>
#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {

/**
 * Evaluates one step of the de Casteljau's algorithm using the given points and
 * the scalar parameter t (class member).
 *
 * @param points A vector of points in 2D
 * @return A vector containing intermediate points or the final interpolated vector
 */
std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points) {
    vector<Vector2D> result = vector<Vector2D>();
    for (int i = 0; i < points.size() - 1; i++) {
        Vector2D p = (1 - this->t) * points[i] + this->t * points[i + 1];
        result.push_back(p);
    }

    return result;
}

/**
 * Evaluates one step of the de Casteljau's algorithm using the given points and
 * the scalar parameter t (function parameter).
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return A vector containing intermediate points or the final interpolated vector
 */
std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const {
    vector<Vector3D> result = vector<Vector3D>();
    for (int i = 0; i < points.size() - 1; i++) {
        Vector3D p = (1 - t) * points[i] + t * points[i + 1];
        result.push_back(p);
    }

    return result;
}

/**
 * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const {
    vector<Vector3D> result = points;
    while (result.size() > 1) {
        result = evaluateStep(result, t);
    }
    return result[0];
}

/**
 * Evaluates the Bezier patch at parameter (u, v)
 *
 * @param u         Scalar interpolation parameter
 * @param v         Scalar interpolation parameter (along the other axis)
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate(double u, double v) const {
    vector<Vector3D> points = vector<Vector3D>();
    for (vector<Vector3D> row : this->controlPoints) {
        points.push_back(evaluate1D(row, u));
    }

    return evaluate1D(points, v);
}

Vector3D Vertex::normal( void ) const {
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    HalfedgeCIter halfedge = this->halfedge();
    Vector3D result(0., 0., 0.);
    double totalArea = 0;
    do {
        FaceCIter face = halfedge->face();

        Vector3D posA = halfedge->vertex()->position;
        Vector3D vecAB = halfedge->next()->vertex()->position - posA;
        Vector3D vecAC = halfedge->next()->next()->vertex()->position - posA;

        // norm(∆ABC) = unit(AB x AC) and area(∆ABC) = .5|AB x AC|. So this is
        // automatically adding the area weighted value with a scaling factor 2,
        // which doesn't matter because we are normalizing at the end anyway.
        result += cross(vecAB, vecAC);

        halfedge = halfedge->twin()->next();
    } while (halfedge != this->halfedge());

    return result.unit();
}

EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 ) {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.

    if (e0->isBoundary()) {
        return e0;
    }

    array<HalfedgeIter, 10> h = {};
    h[0] = e0->halfedge();
    h[1] = h[0]->next();
    h[2] = h[1]->next();
    h[3] = h[0]->twin();
    h[4] = h[3]->next();
    h[5] = h[4]->next();
    h[6] = h[1]->twin();
    h[7] = h[2]->twin();
    h[8] = h[4]->twin();
    h[9] = h[5]->twin();

    array<VertexIter, 4> v = {};
    v[0] = h[0]->vertex();
    v[1] = h[1]->vertex();
    v[2] = h[2]->vertex();
    v[3] = h[5]->vertex();

    array<EdgeIter, 5> e = {};
    e[0] = e0;
    e[1] = h[1]->edge();
    e[2] = h[2]->edge();
    e[3] = h[4]->edge();
    e[4] = h[5]->edge();

    array<FaceIter, 2> f = {};
    f[0] = h[0]->face();
    f[1] = h[3]->face();

    v[0]->halfedge() = h[2];
    v[1]->halfedge() = h[5];
    v[2]->halfedge() = h[1];
    v[3]->halfedge() = h[4];

    e[0]->halfedge() = h[0];
    e[1]->halfedge() = h[5];
    e[2]->halfedge() = h[1];
    e[3]->halfedge() = h[2];
    e[4]->halfedge() = h[4];

    f[0]->halfedge() = h[0];
    f[1]->halfedge() = h[3];

    h[0]->setNeighbors(h[1], h[3], v[3], e[0], f[0]);
    h[1]->setNeighbors(h[2], h[7], v[2], e[2], f[0]);
    h[2]->setNeighbors(h[0], h[8], v[0], e[3], f[0]);
    h[3]->setNeighbors(h[4], h[0], v[2], e[0], f[1]);
    h[4]->setNeighbors(h[5], h[9], v[3], e[4], f[1]);
    h[5]->setNeighbors(h[3], h[6], v[1], e[1], f[1]);
    h[6]->setNeighbors(h[6]->next(), h[5], v[2], e[1], h[6]->face());
    h[7]->setNeighbors(h[7]->next(), h[1], v[0], e[2], h[7]->face());
    h[8]->setNeighbors(h[8]->next(), h[2], v[3], e[3], h[8]->face());
    h[9]->setNeighbors(h[9]->next(), h[4], v[1], e[4], h[9]->face());

    return e[0];
}

VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 ) {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()) {
        return VertexIter();
    }

    // 6 new half edges
    array<HalfedgeIter, 16> h = {};
    h[0] = e0->halfedge();
    h[1] = h[0]->next();
    h[2] = h[1]->next();
    h[3] = h[0]->twin();
    h[4] = h[3]->next();
    h[5] = h[4]->next();
    h[6] = h[1]->twin();
    h[7] = h[2]->twin();
    h[8] = h[4]->twin();
    h[9] = h[5]->twin();
    h[10] = this->newHalfedge();
    h[11] = this->newHalfedge();
    h[12] = this->newHalfedge();
    h[13] = this->newHalfedge();
    h[14] = this->newHalfedge();
    h[15] = this->newHalfedge();

    // 1 new vertices
    array<VertexIter, 5> v = {};
    v[0] = h[0]->vertex();
    v[1] = h[1]->vertex();
    v[2] = h[2]->vertex();
    v[3] = h[5]->vertex();
    v[4] = this->newVertex();

    // 3 new edges
    array<EdgeIter, 8> e = {};
    e[0] = e0;
    e[1] = h[1]->edge();
    e[2] = h[2]->edge();
    e[3] = h[4]->edge();
    e[4] = h[5]->edge();
    e[5] = this->newEdge();
    e[6] = this->newEdge();
    e[7] = this->newEdge();

    // 2 new faces
    array<FaceIter, 4> f = {};
    f[0] = h[0]->face();
    f[1] = h[3]->face();
    f[2] = this->newFace();
    f[3] = this->newFace();

    v[0]->halfedge() = h[4];
    v[1]->halfedge() = h[1];
    v[2]->halfedge() = h[2];
    v[3]->halfedge() = h[5];
    v[4]->halfedge() = h[3];

    e[0]->halfedge() = h[0];
    e[1]->halfedge() = h[1];
    e[2]->halfedge() = h[2];
    e[3]->halfedge() = h[4];
    e[4]->halfedge() = h[5];
    e[5]->halfedge() = h[10];
    e[6]->halfedge() = h[14];
    e[7]->halfedge() = h[11];

    e[7]->isNew = true;
    e[5]->isNew = true;

    f[0]->halfedge() = h[0];
    f[1]->halfedge() = h[3];
    f[2]->halfedge() = h[13];
    f[3]->halfedge() = h[12];

    h[0]->setNeighbors(h[1], h[3], v[4], e[0], f[0]);
    h[1]->setNeighbors(h[10], h[6], v[1], e[1], f[0]);
    h[2]->setNeighbors(h[14], h[7], v[2], e[2], f[2]);
    h[3]->setNeighbors(h[11], h[0], v[1], e[0], f[1]);
    h[4]->setNeighbors(h[12], h[8], v[0], e[3], f[3]);
    h[5]->setNeighbors(h[3], h[9], v[3], e[4], f[1]);
    h[6]->setNeighbors(h[6]->next(), h[1], v[2], e[1], h[6]->face());
    h[7]->setNeighbors(h[7]->next(), h[2], v[0], e[2], h[7]->face());
    h[8]->setNeighbors(h[8]->next(), h[4], v[3], e[3], h[8]->face());
    h[9]->setNeighbors(h[9]->next(), h[5], v[1], e[4], h[9]->face());
    h[10]->setNeighbors(h[0], h[13], v[2], e[5], f[0]);
    h[11]->setNeighbors(h[5], h[12], v[4], e[7], f[1]);
    h[12]->setNeighbors(h[15], h[11], v[3], e[7], f[3]);
    h[13]->setNeighbors(h[2], h[10], v[4], e[5], f[2]);
    h[14]->setNeighbors(h[13], h[15], v[0], e[6], f[2]);
    h[15]->setNeighbors(h[4], h[14], v[4], e[6], f[3]);

    v[4]->position = .5 * (v[1]->position + v[0]->position);
    v[4]->isNew = true;

    return v[4];
}

set<VertexCIter> getEdgeVertices(EdgeCIter edge) {
    set<VertexCIter> result;
    result.insert(edge->halfedge()->vertex());
    result.insert(edge->halfedge()->twin()->vertex());
    return result;
}

void MeshResampler::upsample( HalfedgeMesh& mesh ) {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.

    for (VertexIter vertex = mesh.verticesBegin(); vertex != mesh.verticesEnd(); vertex++) {
        size_t n = vertex->degree();
        float u = n == 3 ? 3. / 16. : 3. / (8. * n);

        Vector3D neighborPositionSum(0., 0., 0.);
        HalfedgeCIter h = vertex->halfedge();
        do {
            VertexCIter vertexNeighbor = h->twin()->vertex();
            neighborPositionSum += vertexNeighbor->position;

            h = h->twin()->next();
        } while (h != vertex->halfedge());

        vertex->newPosition = (1 - n * u) * vertex->position + u * neighborPositionSum;
    }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

    for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
        Vector3D newPosition(0., 0., 0.);

        // Vertex A
        VertexCIter vertex = edge->halfedge()->vertex();
        newPosition += 3. * vertex->position;

        // Vertex B
        vertex = edge->halfedge()->twin()->vertex();
        newPosition += 3. * vertex->position;

        // Vertex C
        vertex = edge->halfedge()->next()->next()->vertex();
        newPosition += vertex->position;

        // Vertex D
        vertex = edge->halfedge()->twin()->next()->next()->vertex();
        newPosition += vertex->position;

        edge->newPosition = newPosition / 8;
    }
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    vector<EdgeIter> edges = vector<EdgeIter>();
    for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
        edge->isNew = false;
        edges.push_back(edge);
    }

    for (VertexIter vertex = mesh.verticesBegin(); vertex != mesh.verticesEnd(); vertex++)
        vertex->isNew = false;

    for (EdgeIter edge : edges) {
        set<VertexCIter> edgeVertices = getEdgeVertices(edge);

        VertexIter vertex = mesh.splitEdge(edge);
        vertex->newPosition = edge->newPosition;
    }
    
    // 4. Flip any new edge that connects an old and new vertex.

    for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
        if (!edge->isNew)
            continue;
        if (edge->halfedge()->vertex()->isNew != edge->halfedge()->twin()->vertex()->isNew) {
            mesh.flipEdge(edge);
        }
    }

    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter vertex = mesh.verticesBegin(); vertex != mesh.verticesEnd(); vertex++) {
        vertex->position = vertex->newPosition;
    }
}

}

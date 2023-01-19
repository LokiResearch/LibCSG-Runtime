using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// CSGBrushOperation
namespace LibCSG{
/// <summary>
/// Instance enum <c>Operation</c> represents the CSG operation.
/// </summary>
public enum Operation {
    OPERATION_UNION,
    OPERATION_INTERSECTION,
    OPERATION_SUBTRACTION,
};

/**
* <summary>
* Class <c>CSGBrushOperation</c> is a class used to do an operation between 2 Brush.
* </summary>
**/
public class CSGBrushOperation
{
    /// <summary>
    /// Instance structure <c>Build2DFaceCollection</c> represents all faces 2D come from the Brush A and the Brush B.
    /// </summary>
    public struct Build2DFaceCollection {
		public Dictionary<int, Build2DFaces> build2DFacesA;
		public Dictionary<int, Build2DFaces> build2DFacesB;
	};

    /// <summary>
    /// This constructor initializes a new CSGBrushOperation.
    /// </summary>
    public CSGBrushOperation(){}

    /// <summary>
    /// This method to do an operation between two Brush
    /// </summary>
    /// <param><c>operation</c> the operation you want to do.</param>
    /// <param><c>brush_a</c> a brush.</param>
    /// <param><c>brush_b</c> another brush.</param>
    /// <param><c>merged_brush</c> a brush to set the result of the operation between brush_a and brush_b.</param>
    /// <param><c>tolerance</c> represents the tolerance used.</param>
    public void merge_brushes(Operation operation, CSGBrush brush_a, CSGBrush brush_b, ref CSGBrush merged_brush, float tolerance = 0.00001f) {
        Build2DFaceCollection build2DFaceCollection;
        build2DFaceCollection.build2DFacesA = new Dictionary<int, Build2DFaces>(brush_a.faces.Length);
        build2DFaceCollection.build2DFacesB = new Dictionary<int, Build2DFaces>(brush_b.faces.Length);
        brush_a.regen_face_aabbs();
        brush_b.regen_face_aabbs();

        for (int i = 0; i < brush_a.faces.Length; i++) {
            for (int j = 0; j < brush_b.faces.Length; j++) {
                if (brush_a.faces[i].aabb.intersects_inclusive(brush_b.faces[j].aabb)) {
                    update_faces(ref brush_a, i, ref brush_b, j, ref build2DFaceCollection, tolerance);
                }
            }
        }

        // Add faces to MeshMerge.
        MeshMerge mesh_merge = new MeshMerge(brush_a.faces.Length + build2DFaceCollection.build2DFacesA.Count,brush_b.faces.Length + build2DFaceCollection.build2DFacesB.Count);
        mesh_merge.vertex_snap = tolerance;
        mesh_merge.scale_a = brush_a.obj.transform.localScale;

        for (int i = 0; i < brush_a.faces.Length; i++) {
            if (build2DFaceCollection.build2DFacesA.ContainsKey(i)) {
                build2DFaceCollection.build2DFacesA[i].addFacesToMesh(ref mesh_merge, false);
            } else {
                Vector3[] points = new Vector3[3];
                Vector2[] uvs = new Vector2[3];
                for (int j = 0; j < 3; j++) {
                    points[j] = brush_a.faces[i].vertices[j];
                    uvs[j] = brush_a.faces[i].uvs[j];
                }
                mesh_merge.add_face(points, uvs, false);
            }
        }

        for (int i = 0; i < brush_b.faces.Length; i++) {
            if (build2DFaceCollection.build2DFacesB.ContainsKey(i)) {
                build2DFaceCollection.build2DFacesB[i].addFacesToMesh(ref mesh_merge, true);
            } else {
                Vector3[] points = new Vector3[3];
                Vector2[] uvs = new Vector2[3];
                for (int j = 0; j < 3; j++) {
                    points[j] = brush_a.obj.transform.InverseTransformPoint(brush_b.obj.transform.TransformPoint(brush_b.faces[i].vertices[j]));
                    uvs[j] = brush_b.faces[i].uvs[j];
                }
                mesh_merge.add_face(points, uvs, true);
            }
        }
        
        Array.Clear(merged_brush.faces, 0 , merged_brush.faces.Length);
        mesh_merge.do_operation(operation, ref merged_brush);
        mesh_merge=null;
        System.GC.Collect();
    }

    
    /// <summary>
    /// This method update the face give
    /// </summary>
    /// <param><c>brush_a</c> a Brush.</param>
    /// <param><c>face_idx_a</c> an id of the face in the first Brush.</param>
    /// <param><c>brush_b</c> another Brush.</param>
    /// <param><c>face_idx_b</c> an id of the face in the second Brush.</param>
    /// <param><c>collection</c> the collection of all face.</param>
    /// <param><c>p_vertex_snap</c> represents the tolerance used.</param>
    void update_faces(ref CSGBrush brush_a, int face_idx_a, ref CSGBrush brush_b, int face_idx_b, ref Build2DFaceCollection collection, float vertex_snap) {
        Vector3[] vertices_a = {
            brush_a.faces[face_idx_a].vertices[0],
            brush_a.faces[face_idx_a].vertices[1],
            brush_a.faces[face_idx_a].vertices[2]
        };

        Vector3[] vertices_b = {
            brush_a.obj.transform.InverseTransformPoint(brush_b.obj.transform.TransformPoint(brush_b.faces[face_idx_b].vertices[0])),
            brush_a.obj.transform.InverseTransformPoint(brush_b.obj.transform.TransformPoint(brush_b.faces[face_idx_b].vertices[1])),
            brush_a.obj.transform.InverseTransformPoint(brush_b.obj.transform.TransformPoint(brush_b.faces[face_idx_b].vertices[2]))
        };

        // Don't use degenerate faces.
        bool has_degenerate = false;
        if (CSGBrush.is_snapable(vertices_a[0], vertices_a[1], vertex_snap) ||
                CSGBrush.is_snapable(vertices_a[0], vertices_a[2], vertex_snap) ||
                CSGBrush.is_snapable(vertices_a[1], vertices_a[2], vertex_snap)) {
            collection.build2DFacesA[face_idx_a] = new Build2DFaces();
            has_degenerate = true;
        }

        if (CSGBrush.is_snapable(vertices_b[0], vertices_b[1], vertex_snap) ||
                CSGBrush.is_snapable(vertices_b[0], vertices_b[2], vertex_snap) ||
                CSGBrush.is_snapable(vertices_b[1], vertices_b[2], vertex_snap)) {
            collection.build2DFacesB[face_idx_b] = new Build2DFaces();
            has_degenerate = true;
        }
        if (has_degenerate) {
            return;
        }

        // Ensure B has points either side of or in the plane of A.
        int over_count = 0, under_count = 0;
        Plane plane_a = new Plane(vertices_a[0], vertices_a[1], vertices_a[2]);
        float distance_tolerance = 0.3f;

        for (int i = 0; i < 3; i++) {
            bool is_point_over = Vector3.Dot(plane_a.normal, vertices_b[i]) > plane_a.distance;
            if (plane_a.GetDistanceToPoint(vertices_b[i]) < distance_tolerance) {
                // In plane.
            } else if (is_point_over) {
                over_count++;
            } else {
                under_count++;
            }
        }
        // If all points under or over the plane, there is no intersection.
        if (over_count == 3 || under_count == 3) {
            return;
        }

        // Ensure A has points either side of or in the plane of B.
        over_count = 0;
        under_count = 0;
        Plane plane_b = new Plane(vertices_b[0], vertices_b[1], vertices_b[2]);

        for (int i = 0; i < 3; i++) {
            bool is_point_over = Vector3.Dot(plane_b.normal, vertices_a[i]) > plane_b.distance;
            if (plane_b.GetDistanceToPoint(vertices_a[i]) < distance_tolerance) {
                // In plane.
            } else if (is_point_over) {
                over_count++;
            } else {
                under_count++;
            }
        }
        // If all points under or over the plane, there is no intersection.
        if (over_count == 3 || under_count == 3) {
            return;
        }

        // Check for intersection using the SAT theorem.
        {
            // Edge pair cross product combinations.
            for (int i = 0; i < 3; i++) {
                Vector3 axis_a = (vertices_a[i] - vertices_a[(i + 1) % 3]);
                axis_a /= axis_a.magnitude;

                for (int j = 0; j < 3; j++) {
                    Vector3 axis_b = (vertices_b[j] - vertices_b[(j + 1) % 3]);
                    axis_b /= axis_b.magnitude;

                    Vector3 sep_axis = Vector3.Cross(axis_a, axis_b);
                    if (sep_axis == Vector3.zero) {
                        continue; //colineal
                    }
                    sep_axis /= sep_axis.magnitude;

                    float min_a = 1e20f, max_a = -1e20f;
                    float min_b = 1e20f, max_b = -1e20f;

                    for (int k = 0; k < 3; k++) {
                        float d = Vector3.Dot(sep_axis, vertices_a[k]);
                        min_a = (min_a < d) ? min_a : d;
                        max_a = (max_a > d) ? max_a : d;
                        d = Vector3.Dot(sep_axis, vertices_b[k]);
                        min_b = (min_b < d) ? min_b : d;
                        max_b = (max_b > d) ? max_b : d;
                    }

                    min_b -= (max_a - min_a) * 0.5f;
                    max_b += (max_a - min_a) * 0.5f;

                    float dmin = min_b - (min_a + max_a) * 0.5f;
                    float dmax = max_b - (min_a + max_a) * 0.5f;

                    if (dmin > CSGBrush.CMP_EPSILON || dmax < -CSGBrush.CMP_EPSILON) {
                        return; // Does not contain zero, so they don't overlap.
                    }
                }
            }
        }

        // If we're still here, the faces probably intersect, so add new faces.
        if (!collection.build2DFacesA.ContainsKey(face_idx_a))
        {
            collection.build2DFacesA.Add(face_idx_a,new Build2DFaces(brush_a, face_idx_a));
        }
        collection.build2DFacesA[face_idx_a].insert(brush_b, face_idx_b, brush_a);

        if (!collection.build2DFacesB.ContainsKey(face_idx_b)) {
            collection.build2DFacesB.Add(face_idx_b, new Build2DFaces(brush_b, face_idx_b, brush_a));
        }
        collection.build2DFacesB[face_idx_b].insert(brush_a, face_idx_a);
    }
    
}
}
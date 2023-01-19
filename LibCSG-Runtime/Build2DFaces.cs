using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace LibCSG{
    /**
    * <summary>
    * Class <c>Build2DFaces</c> is a class used to build 2D Faces and create new faces.
    * </summary>
    **/
    public class Build2DFaces{

        /// <summary>
        /// Instance structure <c>Vertex2D</c> represents a Vertex in 2D.
        /// </summary>
        public struct Vertex2D {
			public Vector2 point;
			public Vector2 uv;
		};

        /// <summary>
        /// Instance structure <c>Face2D</c> represents a Face in 2D.
        /// </summary>
		public struct Face2D {
			public int[] vertex_idx;
		};

        /// <summary>
        /// Instance variable <c>vertices</c> represents a all of vertex2D.
        /// </summary>
        private List<Vertex2D> vertices = new List<Vertex2D>();
        /// <summary>
        /// Instance variable <c>faces</c> represents a all of Face2D.
        /// </summary>
		private List<Face2D> faces = new List<Face2D>();

        /// <summary>
        /// Instance variable <c>vertex_tolerance</c> represents the tolerance used.
        /// </summary>
		private float vertex_tolerance = 1e-10f;

        /// <summary>
        /// Instance variable <c>to_3D</c> represents transform to transform a 3D point to a 2D point.
        /// </summary>
		private Transform_to_2DFace to_3D;

        /// <summary>
        /// Instance variable <c>to_2D</c> represents transform to transform a 2D point to a 3D point.
        /// </summary>
		private Transform_to_2DFace to_2D;
        /// <summary>
        /// Instance variable <c>plane</c> used in the transform.
        /// </summary>
		private PlaneCSG plane;

        /// <summary>
        /// This static method get the point in the segment closest the point given in parameters
        /// </summary>
        /// <param><c>point</c> is a point you want get the closest point in the segment.</param>
        /// <param><c>segment</c> an array with 2 vector2 corresponding to the first and last point of the segment.</param>
        /// <returns>
        /// Return a point in the segment closest the point given in parameters.
        /// </returns>
        static Vector2 get_closest_point_to_segment(Vector2 point, Vector2[] segment) {
            Vector2 p = point - segment[0];
            Vector2 n = segment[1] - segment[0];
            float l2 = n.sqrMagnitude;
            if (l2 < 1e-20f) {
                return segment[0]; // Both points are the same, just give any.
            }

            float d = Vector2.Dot(n, p) / l2;

            if (d <= 0.0f) {
                return segment[0]; // Before first point.
            } else if (d >= 1.0f) {
                return segment[1]; // After first point.
            } else {
                return segment[0] + n * d; // Inside.
            }
        }

        /// <summary>
        /// This static method check if 2 segments intersect and if it is true, this methode calculate the intersection point
        /// </summary>
        /// <param><c>from_a</c> the point where first segment start.</param>
        /// <param><c>to_a</c> the point where first segment end.</param>
        /// <param><c>from_b</c> the point where second segment start.</param>
        /// <param><c>to_b</c> the point where second segment end.</param>
        /// <param><c>result</c> a Vector2 to set the intersection point between the two segment.</param>
        /// <returns>
        /// Return True if 2 segments intersect and put the intersection point else return False
        /// </returns>
        static bool segment_intersects_segment(Vector2 from_a, Vector2 to_a, Vector2 from_b, Vector2 to_b, ref Vector2 result) {
            Vector2 AB = to_a - from_a;
            Vector2 AC = from_b - from_a;
            Vector2 AD = to_b - from_a;

            float ABlen = Vector2.Dot(AB,AB);
            if (ABlen <= 0) {
                return false;
            }
            Vector2 AB_norm = AB / ABlen;
            AC = new Vector2(AC.x * AB_norm.x + AC.y * AB_norm.y, AC.y * AB_norm.x - AC.x * AB_norm.y);
            AD = new Vector2(AD.x * AB_norm.x + AD.y * AB_norm.y, AD.y * AB_norm.x - AD.x * AB_norm.y);

            // segments don't intersect
            if ((AC.y < (float)-CSGBrush.CMP_EPSILON && AD.y < (float)-CSGBrush.CMP_EPSILON) || (AC.y > (float)CSGBrush.CMP_EPSILON && AD.y > (float)CSGBrush.CMP_EPSILON)) {
                return false;
            }

            if (Mathf.Abs(AD.y - AC.y) < CSGBrush.CMP_EPSILON) {
                return false;
            }

            float ABpos = AD.x + (AC.x - AD.x) * AD.y / (AD.y - AC.y);

            if ((ABpos < 0) || (ABpos > 1)) {
                return false;
            }

            result = from_a + AB * ABpos;
            return true;
        }

        /// <summary>
        /// This static method check if 2 segments are parallel.
        /// </summary>
        /// <param><c>segment1_points</c> an array with 2 vector2 corresponding to the first and last point of the segment.</param>
        /// <param><c>segment2_points</c> an array with 2 vector2 corresponding to the first and last point of the segment.</param>
        /// <param><c>tolerance</c> is a tolerance you want used.</param>
        /// <returns>
        /// Return True if 2 segments are parallel else return False
        /// </returns>
        static bool are_segments_parallel(Vector2[] segment1_points, Vector2[] segment2_points, float tolerance) {
            Vector2 segment1 = segment1_points[1] - segment1_points[0];
            Vector2 segment2 = segment2_points[1] - segment2_points[0];
            float segment1_length2 = Vector2.Dot(segment1, segment1);
            float segment2_length2 = Vector2.Dot(segment2, segment2);
            float segment_onto_segment = Vector2.Dot(segment2, segment1);

            if (segment1_length2 < tolerance || segment2_length2 < tolerance) {
                return true;
            }

            float max_separation2;
            if (segment1_length2 > segment2_length2) {
                max_separation2 = segment2_length2 - segment_onto_segment * segment_onto_segment / segment1_length2;
            } else {
                max_separation2 = segment1_length2 - segment_onto_segment * segment_onto_segment / segment2_length2;
            }

            return max_separation2 < tolerance;
        }

        /// <summary>
        /// This static method check if the point given in the parameters is inside the triangle.
        /// </summary>
        /// <param><c>point</c> a point you want know if it is inside the triangle.</param>
        /// <param><c>a</c> is a vertex of the triangle.</param>
        /// <param><c>b</c> is a vertex of the triangle.</param>
        /// <param><c>c</c> is a vertex of the triangle.</param>
        /// <returns>
        /// Return True if the point given in the parameters is inside the triangle else return False
        /// </returns>
        static bool is_point_in_triangle(Vector2 point, Vector2 a,  Vector2 b,  Vector2 c) {
            Vector2 an = a - point;
            Vector2 bn = b - point;
            Vector2 cn = c - point;

            bool orientation = (an.x * bn.y - an.y * bn.x) > 0;

            if (((bn.x * cn.y - bn.y * cn.x) > 0) != orientation) {
                return false;
            }

            return ((cn.x * an.y - cn.y * an.x) > 0) == orientation;
        }

        /// <summary>
        /// This static method check if the segment intersect the plane.
        /// </summary>
        /// <param><c>plane</c> a plane.</param>
        /// <param><c>begin</c> the point where segment start.</param>
        /// <param><c>end</c> the point where segment end.</param>
        /// <param><c>result</c> a Vector3 to set the intersection point between the segment and the plane.</param>
        /// <returns>
        /// Return True if 2 segments intersect and put the intersection point else return False
        /// </returns>
        static bool plane_intersects_segment(PlaneCSG plane, Vector3 begin, Vector3 end, ref Vector3 result){
            Vector3 segment = begin - end;
            float den = Vector3.Dot(plane.normal, segment);

            if (Mathf.Abs(den) < CSGBrush.CMP_EPSILON) {
                return false;
            }

            float dist = (Vector3.Dot(plane.normal, begin) - plane.d) / den;

            if (dist < (float)-CSGBrush.CMP_EPSILON || dist > (1.0f + (float)CSGBrush.CMP_EPSILON)) {
                return false;
            }

            dist = -dist;
            result = begin + segment * dist;

            return true;
        }

        /// <summary>
        /// This constructor initializes a new Build2DFaces.
        /// </summary>
        public Build2DFaces() {}

        /// <summary>
        /// This constructor initializes a new Build2DFaces with a CSGBrush, an id to get a face in the CSGBrush.
        /// </summary>
        /// <param><c>brush</c> is a brush.</param>
        /// <param><c>face_idx</c> is a the id of a face in the brush.</param>
        public Build2DFaces(CSGBrush brush, int face_idx) {
            // Convert 3D vertex points to 2D.
            Vector3[] points_3D = {
                brush.faces[face_idx].vertices[0],
                brush.faces[face_idx].vertices[1],
                brush.faces[face_idx].vertices[2]
            };
            plane = new PlaneCSG(points_3D[0], points_3D[1], points_3D[2]);
            to_3D = new Transform_to_2DFace();
            to_3D.Set_position(points_3D[0]);
            to_3D.basis_set_column(2, plane.normal);
            Vector3 temp = points_3D[1] - points_3D[2];
            temp /= temp.magnitude;
            to_3D.basis_set_column(0, temp);
            temp = Vector3.Cross(to_3D.basis_get_column(0), to_3D.basis_get_column(2));
            temp /= temp.magnitude;
            to_3D.basis_set_column(1, temp);
            to_2D = to_3D.affine_inverse();

            Face2D face;
            face.vertex_idx = new int[3];
            for (int i = 0; i < 3; i++) {
                Vertex2D vertex;
                Vector3 point_2D = to_2D.xform(points_3D[i]);
                vertex.point.x = point_2D.x;
                vertex.point.y = point_2D.y;
                vertex.uv = brush.faces[face_idx].uvs[i];
                vertices.Add(vertex);
                face.vertex_idx[i] = i;
            }
            faces.Add(face);
        }
       
        /// <summary>
        /// This constructor initializes a new Build2DFaces with a CSGBrush, an id to get a face in the CSGBrush. The Build2DFaces is create in the Transform of brush_a
        /// </summary>
        /// <param><c>brush</c> is a brush.</param>
        /// <param><c>face_idx</c> is a the id of a face in the brush.</param>
        /// <param><c>brush_a</c> is a the brush containing the Transform you want used.</param>
        public Build2DFaces(CSGBrush brush, int face_idx, CSGBrush brush_a) {
            // Convert 3D vertex points to 2D.
            Vector3[] points_3D = {
                brush_a.obj.transform.InverseTransformPoint(brush.obj.transform.TransformPoint(brush.faces[face_idx].vertices[0])),
                brush_a.obj.transform.InverseTransformPoint(brush.obj.transform.TransformPoint(brush.faces[face_idx].vertices[1])),
                brush_a.obj.transform.InverseTransformPoint(brush.obj.transform.TransformPoint(brush.faces[face_idx].vertices[2]))
            };
            plane = new PlaneCSG(points_3D[0], points_3D[1], points_3D[2]);
            to_3D = new Transform_to_2DFace();
            to_3D.Set_position(points_3D[0]);
            to_3D.basis_set_column(2, plane.normal);
            Vector3 temp = points_3D[1] - points_3D[2];
            temp /= temp.magnitude;
            to_3D.basis_set_column(0, temp);
            temp = Vector3.Cross(to_3D.basis_get_column(0), to_3D.basis_get_column(2));
            temp /= temp.magnitude;
            to_3D.basis_set_column(1, temp);
            to_2D = to_3D.affine_inverse();

            Face2D face;
            face.vertex_idx = new int[3];
            for (int i = 0; i < 3; i++) {
                Vertex2D vertex;
                Vector3 point_2D = to_2D.xform(points_3D[i]);
                vertex.point.x = point_2D.x;
                vertex.point.y = point_2D.y;
                vertex.uv = brush.faces[face_idx].uvs[i];
                vertices.Add(vertex);
                face.vertex_idx[i] = i;
            }
            faces.Add(face);
        }

        /// <summary>
        /// This method check if the point is already in the vertice list and return the id of the vertex
        /// </summary>
        /// <param><c>point</c> is a point you want know if it is in the vertice list.</param>
        /// <returns>
        /// Return the id in the vertice list corresponding to the point if point is not in the list the methode return -1.
        /// </returns>
        private int get_point_idx(Vector2 point) {
            for (int vertex_idx = 0; vertex_idx < vertices.Count; ++vertex_idx) {
                if ((vertices[vertex_idx].point - point).sqrMagnitude < vertex_tolerance) {
                    return vertex_idx;
                }
            }
            return -1;
        }

        /// <summary>
        /// This method add the vertex given in parameters
        /// </summary>
        /// <param><c>vertex</c> is a point you want know if it is in the vertice list.</param>
        /// <returns>
        /// Return the id of the vertex2D add in the vertices List.
        /// </returns>
        private int add_vertex(Vertex2D vertex) {
            // Check if vertex exists.
            int vertex_id = get_point_idx(vertex.point);
            if (vertex_id != -1) {
                return vertex_id;
            }

            vertices.Add(vertex);
            return vertices.Count - 1;
        }


        /// <summary>
        /// This method add the id vertex given in parameters in a list sorted by the along the axis with the greatest difference
        /// </summary>
        /// <param><c>vertex_indices</c> a list of indice vertice.</param>
        /// <param><c>new_vertex_index</c> a index vertice youwant add in the list.</param>
        private void add_vertex_idx_sorted(List<int> vertex_indices, int new_vertex_index) {
            if (new_vertex_index >= 0 && vertex_indices.IndexOf(new_vertex_index) == -1) {

                // The first vertex.
                if (vertex_indices.Count == 0) {
                    vertex_indices.Add(new_vertex_index);
                    return;
                }

                // The second vertex.
                Vector2 first_point;
                Vector2 new_point;
                int axis;
                if (vertex_indices.Count == 1) {
                    first_point = vertices[vertex_indices[0]].point;
                    new_point = vertices[new_vertex_index].point;

                    // Sort along the axis with the greatest difference.
                    axis = 0;
                    if (Mathf.Abs(new_point.x - first_point.x) < Mathf.Abs(new_point.y - first_point.y)) {
                        axis = 1;
                    }

                    // Add it to the beginning or the end appropriately.
                    if (new_point[axis] < first_point[axis]) {
                        vertex_indices.Insert(0, new_vertex_index);
                    } else {
                        vertex_indices.Add(new_vertex_index);
                    }

                    return;
                }

                // Third vertices.
                first_point = vertices[vertex_indices[0]].point;
                Vector2 last_point = vertices[vertex_indices[vertex_indices.Count - 1]].point;
                new_point = vertices[new_vertex_index].point;

                // Determine axis being sorted against i.e. the axis with the greatest difference.
                axis = 0;
                if (Mathf.Abs(last_point.x - first_point.x) < Mathf.Abs(last_point.y - first_point.y)) {
                    axis = 1;
                }

                // Insert the point at the appropriate index.
                for (int insert_idx = 0; insert_idx < vertex_indices.Count; ++insert_idx) {
                    Vector2 insert_point = vertices[vertex_indices[insert_idx]].point;
                    if (new_point[axis] < insert_point[axis]) {
                        vertex_indices.Insert(insert_idx, new_vertex_index);
                        return;
                    }
                }
                // New largest, add it to the end.
                vertex_indices.Add(new_vertex_index);
            }
        }

        /// <summary>
        /// This method merge the faces of build2DFaces with the face given in parameters
        /// </summary>
        /// <param><c>segment_indices</c> a list containing indices of vertice create that composes the face.</param>
        private void merge_faces(List<int> segment_indices) {
            int segments = segment_indices.Count - 1;
            if (segments < 2) {
                return;
            }

            // Faces around an inner vertex are merged by moving the inner vertex to the first vertex.
            for (int sorted_idx = 1; sorted_idx < segments; ++sorted_idx) {
                int closest_idx = 0;
                int inner_idx = segment_indices[sorted_idx];

                if (sorted_idx > segments / 2) {
                    // Merge to other segment end.
                    closest_idx = segments;
                    // Reverse the merge order.
                    inner_idx = segment_indices[segments + segments / 2 - sorted_idx];
                }

                // Find the mergeable faces.
                List<int> merge_faces_idx = new List<int>();
                List<Face2D> merge_faces = new List<Face2D>();
                List<int> merge_faces_inner_vertex_idx = new List<int>();
                for (int face_idx = 0; face_idx < faces.Count; ++face_idx) {
                    for (int face_vertex_idx = 0; face_vertex_idx < 3; ++face_vertex_idx) {
                        if (faces[face_idx].vertex_idx[face_vertex_idx] == inner_idx) {
                            merge_faces_idx.Add(face_idx);
                            merge_faces.Add(faces[face_idx]);
                            merge_faces_inner_vertex_idx.Add(face_vertex_idx);
                        }
                    }
                }

                List<int> degenerate_points = new List<int>();

                // Create the new faces.
                for (int merge_idx = 0; merge_idx < merge_faces.Count; ++merge_idx) {
                    int[] outer_edge_idx = new int[2];
                    outer_edge_idx[0] = merge_faces[merge_idx].vertex_idx[(merge_faces_inner_vertex_idx[merge_idx] + 1) % 3];
                    outer_edge_idx[1] = merge_faces[merge_idx].vertex_idx[(merge_faces_inner_vertex_idx[merge_idx] + 2) % 3];

                    // Skip flattened faces.
                    if (outer_edge_idx[0] == segment_indices[closest_idx] ||
                            outer_edge_idx[1] == segment_indices[closest_idx]) {
                        continue;
                    }

                    //Don't create degenerate triangles.
                    Vector2[] edge1 = {
                        vertices[outer_edge_idx[0]].point,
                        vertices[segment_indices[closest_idx]].point
                    };
                    Vector2[] edge2 = {
                        vertices[outer_edge_idx[1]].point,
                        vertices[segment_indices[closest_idx]].point
                    };
                    if (CSGBrush.are_segments_parallel(edge1, edge2, vertex_tolerance)) {
                        if (!degenerate_points.Contains(outer_edge_idx[0])) {
                            degenerate_points.Add(outer_edge_idx[0]);
                        }
                        if (!degenerate_points.Contains(outer_edge_idx[1])) {
                            degenerate_points.Add(outer_edge_idx[1]);
                        }
                        continue;
                    }

                    // Create new faces.
                    Face2D new_face;
                    new_face.vertex_idx = new int[3];
                    new_face.vertex_idx[0] = segment_indices[closest_idx];
                    new_face.vertex_idx[1] = outer_edge_idx[0];
                    new_face.vertex_idx[2] = outer_edge_idx[1];
                    faces.Add(new_face);
                }

                // Delete the old faces in reverse index order.
                merge_faces_idx.Sort();
                merge_faces_idx.Reverse();
                for (int i = 0; i < merge_faces_idx.Count; ++i) {
                    faces.RemoveAt(merge_faces_idx[i]);
                }

                if (degenerate_points.Count == 0) {
                    continue;
                }

                // Split faces using degenerate points.
                for (int face_idx = 0; face_idx < faces.Count; ++face_idx) {
                    Face2D face = faces[face_idx];
                    Vertex2D[] face_vertices = {
                        vertices[face.vertex_idx[0]],
                        vertices[face.vertex_idx[1]],
                        vertices[face.vertex_idx[2]]
                    };
                    Vector2[] face_points = {
                        face_vertices[0].point,
                        face_vertices[1].point,
                        face_vertices[2].point
                    };

                    for (int point_idx = 0; point_idx < degenerate_points.Count; ++point_idx) {
                        int degenerate_idx = degenerate_points[point_idx];
                        Vector2 point_2D = vertices[degenerate_idx].point;

                        // Check if point is existing face vertex.
                        bool existing = false;
                        for (int i = 0; i < 3; ++i) {
                            if ((face_vertices[i].point - point_2D).sqrMagnitude < vertex_tolerance) {
                                existing = true;
                                break;
                            }
                        }
                        if (existing) {
                            continue;
                        }

                        // Check if point is on each edge.
                        for (int face_edge_idx = 0; face_edge_idx < 3; ++face_edge_idx) {
                            Vector2[] edge_points = {
                                face_points[face_edge_idx],
                                face_points[(face_edge_idx + 1) % 3]
                            };
                            Vector2 closest_point = get_closest_point_to_segment(point_2D, edge_points);

                            if ((point_2D - closest_point ).sqrMagnitude < vertex_tolerance) {
                                int opposite_vertex_idx = face.vertex_idx[(face_edge_idx + 2) % 3];

                                // If new vertex snaps to degenerate vertex, just delete this face.
                                if (degenerate_idx == opposite_vertex_idx) {
                                    faces.RemoveAt(face_idx);
                                    // Update index.
                                    --face_idx;
                                    break;
                                }

                                // Create two new faces around the new edge and remove this face.
                                // The new edge is the last edge.
                                Face2D left_face;
                                left_face.vertex_idx = new int[3];
                                left_face.vertex_idx[0] = degenerate_idx;
                                left_face.vertex_idx[1] = face.vertex_idx[(face_edge_idx + 1) % 3];
                                left_face.vertex_idx[2] = opposite_vertex_idx;
                                Face2D right_face;
                                right_face.vertex_idx = new int[3];
                                right_face.vertex_idx[0] = opposite_vertex_idx;
                                right_face.vertex_idx[1] = face.vertex_idx[face_edge_idx];
                                right_face.vertex_idx[2] = degenerate_idx;
                                faces.RemoveAt(face_idx);
                                faces.Insert(face_idx, right_face);
                                faces.Insert(face_idx, left_face);

                                // Don't check against the new faces.
                                ++face_idx;

                                // No need to check other edges.
                                break;
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// This method find the edge that intersect the segment
        /// </summary>
        /// <param><c>segment_points</c> an array with 2 vector2 corresponding to the first and last point of the segment.</param>
        /// <param><c>segment_indices</c> a list to set the new indice.</param>
        private void find_edge_intersections(Vector2[] segment_points, ref List<int> segment_indices) {
            // For each face.
            for (int face_idx = 0; face_idx < faces.Count; ++face_idx) {
                Face2D face = faces[face_idx];
                Vertex2D[] face_vertices = {
                    vertices[face.vertex_idx[0]],
                    vertices[face.vertex_idx[1]],
                    vertices[face.vertex_idx[2]]
                };

                // Check each edge.
                for (int face_edge_idx = 0; face_edge_idx < 3; ++face_edge_idx) {
                    Vector2[] edge_points = {
                        face_vertices[face_edge_idx].point,
                        face_vertices[(face_edge_idx + 1) % 3].point
                    };
                    Vector2[] edge_uvs = {
                        face_vertices[face_edge_idx].uv,
                        face_vertices[(face_edge_idx + 1) % 3].uv
                    };
                    Vector2 intersection_point = Vector2.zero;

                    // First check if the ends of the segment are on the edge.
                    bool on_edge = false;
                    for (int edge_point_idx = 0; edge_point_idx < 2; ++edge_point_idx) {
                        intersection_point = get_closest_point_to_segment(segment_points[edge_point_idx], edge_points);
                        if ((segment_points[edge_point_idx] - intersection_point).sqrMagnitude < vertex_tolerance) {
                            on_edge = true;
                            break;
                        }
                    }

                    // Else check if the segment intersects the edge.
                    if (on_edge || segment_intersects_segment(segment_points[0], segment_points[1], edge_points[0], edge_points[1], ref intersection_point)) {
                        // Check if intersection point is an edge point.
                        if (((edge_points[0] - intersection_point).sqrMagnitude < vertex_tolerance) ||
                                ((edge_points[1] - intersection_point).sqrMagnitude < vertex_tolerance)) {
                            continue;
                        }

                        // Check if edge exists, by checking if the intersecting segment is parallel to the edge.
                        if (are_segments_parallel(segment_points, edge_points, vertex_tolerance)) {
                            continue;
                        }

                        // Add the intersection point as a new vertex.
                        Vertex2D new_vertex;
                        new_vertex.point = intersection_point;
                        new_vertex.uv = CSGBrush.interpolate_segment_uv(edge_points, edge_uvs, intersection_point);
                        int new_vertex_idx = add_vertex(new_vertex);
                        int opposite_vertex_idx = face.vertex_idx[(face_edge_idx + 2) % 3];
                        add_vertex_idx_sorted(segment_indices, new_vertex_idx);

                        // If new vertex snaps to opposite vertex, just delete this face.
                        if (new_vertex_idx == opposite_vertex_idx) {
                            faces.RemoveAt(face_idx);
                            // Update index.
                            --face_idx;
                            break;
                        }

                        // If opposite point is on the segment, add its index to segment indices too.
                        Vector2 closest_point = get_closest_point_to_segment(vertices[opposite_vertex_idx].point, segment_points);
                        if ((vertices[opposite_vertex_idx].point - closest_point).sqrMagnitude < vertex_tolerance) {
                            add_vertex_idx_sorted(segment_indices, opposite_vertex_idx);
                        }

                        // Create two new faces around the new edge and remove this face.
                        // The new edge is the last edge.
                        Face2D left_face;
                        left_face.vertex_idx = new int[3];
                        left_face.vertex_idx[0] = new_vertex_idx;
                        left_face.vertex_idx[1] = face.vertex_idx[(face_edge_idx + 1) % 3];
                        left_face.vertex_idx[2] = opposite_vertex_idx;
                        Face2D right_face;
                        right_face.vertex_idx = new int[3];
                        right_face.vertex_idx[0] = opposite_vertex_idx;
                        right_face.vertex_idx[1] = face.vertex_idx[face_edge_idx];
                        right_face.vertex_idx[2] = new_vertex_idx;
                        faces.RemoveAt(face_idx);
                        faces.Insert(face_idx, right_face);
                        faces.Insert(face_idx, left_face);

                        // Check against the new faces.
                        --face_idx;
                        break;
                    }
                }
            }
        }

        /// <summary>
        /// This method insert point in the vertices.
        /// </summary>
        /// <param><c>point</c> the point you want add in the vertices list.</param>
        /// <returns>
        /// Return the id of the vertex2D in the vertices List.
        /// </returns>
        private int insert_point(Vector2 point) {
            int new_vertex_idx = -1;

            for (int face_idx = 0; face_idx < faces.Count; ++face_idx) {
                Face2D face = faces[face_idx];
                Vertex2D[] face_vertices = {
                    vertices[face.vertex_idx[0]],
                    vertices[face.vertex_idx[1]],
                    vertices[face.vertex_idx[2]]
                };
                Vector2[] points = {
                    face_vertices[0].point,
                    face_vertices[1].point,
                    face_vertices[2].point
                };
                Vector2[] uvs = {
                    face_vertices[0].uv,
                    face_vertices[1].uv,
                    face_vertices[2].uv
                };

                // Skip degenerate triangles.
                if (CSGBrush.is_triangle_degenerate(points, vertex_tolerance)) {
                    continue;
                }

                // Check if point is existing face vertex.
                for (int i = 0; i < 3; ++i) {
                    if ((face_vertices[i].point - point).sqrMagnitude < vertex_tolerance) {
                        return face.vertex_idx[i];
                    }
                }

                // Check if point is on each edge.
                bool on_edge = false;
                for (int face_edge_idx = 0; face_edge_idx < 3; ++face_edge_idx) {
                    Vector2[] edge_points = {
                        points[face_edge_idx],
                        points[(face_edge_idx + 1) % 3]
                    };
                    Vector2[] edge_uvs = {
                        uvs[face_edge_idx],
                        uvs[(face_edge_idx + 1) % 3]
                    };

                    Vector2 closest_point = get_closest_point_to_segment(point, edge_points);
                    if ((point - closest_point).sqrMagnitude < vertex_tolerance) {
                        on_edge = true;

                        // Add the point as a new vertex.
                        Vertex2D new_vertex;
                        new_vertex.point = point;
                        new_vertex.uv = CSGBrush.interpolate_segment_uv(edge_points, edge_uvs, point);
                        new_vertex_idx = add_vertex(new_vertex);
                        int opposite_vertex_idx = face.vertex_idx[(face_edge_idx + 2) % 3];

                        // If new vertex snaps to opposite vertex, just delete this face.
                        if (new_vertex_idx == opposite_vertex_idx) {
                            faces.RemoveAt(face_idx);
                            // Update index.
                            --face_idx;
                            break;
                        }

                        // Don't create degenerate triangles.
                        Vector2[] split_edge1 = { vertices[new_vertex_idx].point, edge_points[0] };
                        Vector2[] split_edge2 = { vertices[new_vertex_idx].point, edge_points[1] };
                        Vector2[] new_edge = { vertices[new_vertex_idx].point, vertices[opposite_vertex_idx].point };
                        if (are_segments_parallel(split_edge1, new_edge, vertex_tolerance) &&
                                are_segments_parallel(split_edge2, new_edge, vertex_tolerance)) {
                            break;
                        }

                        // Create two new faces around the new edge and remove this face.
                        // The new edge is the last edge.
                        Face2D left_face;
                        left_face.vertex_idx = new int[3];
                        left_face.vertex_idx[0] = new_vertex_idx;
                        left_face.vertex_idx[1] = face.vertex_idx[(face_edge_idx + 1) % 3];
                        left_face.vertex_idx[2] = opposite_vertex_idx;
                        Face2D right_face;
                        right_face.vertex_idx = new int[3];
                        right_face.vertex_idx[0] = opposite_vertex_idx;
                        right_face.vertex_idx[1] = face.vertex_idx[face_edge_idx];
                        right_face.vertex_idx[2] = new_vertex_idx;
                        faces.RemoveAt(face_idx);
                        faces.Insert(face_idx, right_face);
                        faces.Insert(face_idx, left_face);

                        // Don't check against the new faces.
                        ++face_idx;

                        // No need to check other edges.
                        break;
                    }
                }

                // If not on an edge, check if the point is inside the face.
                if (!on_edge && is_point_in_triangle(point, face_vertices[0].point, face_vertices[1].point, face_vertices[2].point)) {
                    // Add the point as a new vertex.
                    Vertex2D new_vertex;
                    new_vertex.point = point;
                    new_vertex.uv = CSGBrush.interpolate_triangle_uv(points, uvs, point);
                    new_vertex_idx = add_vertex(new_vertex);

                    // Create three new faces around this point and remove this face.
                    // The new vertex is the last vertex.
                    for (int i = 0; i < 3; ++i) {
                        // Don't create degenerate triangles.
                        Vector2[] new_points = { points[i], points[(i + 1) % 3], vertices[new_vertex_idx].point };
                        if (CSGBrush.is_triangle_degenerate(new_points, vertex_tolerance)) {
                            continue;
                        }

                        Face2D new_face;
                        new_face.vertex_idx = new int[3];
                        new_face.vertex_idx[0] = face.vertex_idx[i];
                        new_face.vertex_idx[1] = face.vertex_idx[(i + 1) % 3];
                        new_face.vertex_idx[2] = new_vertex_idx;
                        faces.Add(new_face);
                    }
                    faces.RemoveAt(face_idx);

                    // No need to check other faces.
                    break;
                }
            }

            return new_vertex_idx;
        }

        /// <summary>
        /// This method insert the face in the Build2DFaces.
        /// </summary>
        /// <param><c>brush</c> the Brush containing the face you want add.</param>
        /// <param><c>face_idx</c> the id of the face you want add.</param>
        public void insert(CSGBrush brush, int face_idx, CSGBrush brush_a = null) {
            Vector2[] points_2D = new Vector2[3];
            int points_count = 0;

            for (int i = 0; i < 3; i++) {
                Vector3 point_3D;
                if(brush_a==null){
                    point_3D = brush.faces[face_idx].vertices[i];
                }
                else{
                    point_3D = brush_a.obj.transform.InverseTransformPoint(brush.obj.transform.TransformPoint(brush.faces[face_idx].vertices[i]));
                }

                if (plane.has_point(point_3D, CSGBrush.CMP_EPSILON)) {
                    // Point is in the plane, add it.
                    Vector3 point_2D = Vector3.ProjectOnPlane(point_3D , plane.normal);
                    point_2D = to_2D.xform(point_2D);
                    points_2D[points_count++] = new Vector2(point_2D.x, point_2D.y);

                } else {
                    Vector3 next_point_3D;
                    if(brush_a==null){
                        next_point_3D = brush.faces[face_idx].vertices[(i + 1) % 3];
                    }
                    else{
                        next_point_3D = brush_a.obj.transform.InverseTransformPoint(brush.obj.transform.TransformPoint(brush.faces[face_idx].vertices[(i + 1) % 3]));
                    }

                    if (plane.has_point(next_point_3D, CSGBrush.CMP_EPSILON)) {
                        continue; // Next point is in plane, it will be added separately.
                    }
                    if (plane.is_point_over(point_3D) == plane.is_point_over(next_point_3D)) {
                        continue; // Both points on the same side of the plane, ignore.
                    }

                    // Edge crosses the plane, find and add the intersection point.
                    Vector3 point_2D= Vector3.zero;
                    if (plane_intersects_segment(plane, point_3D, next_point_3D, ref point_2D)) {
                        point_2D = to_2D.xform(point_2D);
                        points_2D[points_count++] = new Vector2(point_2D.x, point_2D.y);
                    }
                }
            }

            List<int> segment_indices = new List<int>();
            Vector2[] segment = new Vector2[2];
            int[] inserted_index = { -1, -1, -1 };

            // Insert points.
            for (int i = 0; i < points_count; ++i) {
                inserted_index[i] = insert_point(points_2D[i]);
            }

            if (points_count == 2) {
                // Insert a single segment.
                segment[0] = points_2D[0];
                segment[1] = points_2D[1];
                find_edge_intersections(segment, ref segment_indices);
                for (int i = 0; i < 2; ++i) {
                    add_vertex_idx_sorted(segment_indices, inserted_index[i]);
                }
                merge_faces(segment_indices);
            }

            if (points_count == 3) {
                // Insert three segments.
                for (int edge_idx = 0; edge_idx < 3; ++edge_idx) {
                    segment[0] = points_2D[edge_idx];
                    segment[1] = points_2D[(edge_idx + 1) % 3];
                    find_edge_intersections(segment, ref segment_indices);
                    for (int i = 0; i < 2; ++i) {
                        add_vertex_idx_sorted(segment_indices, inserted_index[(edge_idx + i) % 3]);
                    }
                    merge_faces(segment_indices);
                    segment_indices.Clear();
                }
            }
        }

        /// <summary>
        /// This method add all faces in the mesh merge.
        /// </summary>
        /// <param><c>mesh_merge</c> the mesh merge where you want add the mesh.</param>
        /// <param><c>from_b</c> a bool True if the faces come from the Brush B .</param>
        public void addFacesToMesh(ref MeshMerge mesh_merge, bool from_b) {
            for (int face_idx = 0; face_idx < faces.Count; ++face_idx) {
                Face2D face = faces[face_idx];
                Vertex2D[] fv = {
                    vertices[face.vertex_idx[0]],
                    vertices[face.vertex_idx[1]],
                    vertices[face.vertex_idx[2]]
                };

                // Convert 2D vertex points to 3D.
                Vector3[] points_3D = new Vector3[3];
                Vector2[] uvs = new Vector2[3];
                for (int i = 0; i < 3; ++i) {
                    Vector3 point_2D = new Vector3(fv[i].point.x, fv[i].point.y, 0);
                    points_3D[i] = to_3D.xform(point_2D);
                    uvs[i] = fv[i].uv;
                }

                mesh_merge.add_face(points_3D, uvs, from_b);
            }
        }
    }
}
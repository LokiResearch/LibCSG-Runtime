using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace LibCSG{
/**
* <summary>
* Class <c>MeshMerge</c> is a class used to merge mesh.
* </summary>
**/
public class MeshMerge{
        // Use a limit to speed up bvh and limit the depth.
        static int BVH_LIMIT = 10;

        enum VISIT {
            TEST_AABB_BIT = 0,
            VISIT_LEFT_BIT = 1,
            VISIT_RIGHT_BIT = 2,
            VISIT_DONE_BIT = 3,
            VISITED_BIT_SHIFT = 29,
            NODE_IDX_MASK = (1 << VISITED_BIT_SHIFT) - 1,
            VISITED_BIT_MASK = ~NODE_IDX_MASK
        };

        /// <summary>
        /// Instance structure <c>FaceBVH</c> represents face to the BVH representation.
        /// </summary>
		public struct FaceBVH {
			public int face;
			public int left;
			public int right;
			public int next;
			public Vector3 center;
			public AABB aabb;
		};

        /// <summary>
        /// Instance structure <c>Face</c> represents face of the mesh merge.
        /// </summary>
        public struct Face {
			public bool from_b;
			public int[] points;
			public Vector2[] uvs;
		};
        
        /// <summary>
        /// Instance variable <c>points</c> a list of vertices of the mesh.
        /// </summary>
		public List<Vector3> points;  

        /// <summary>
        /// Instance variable <c>faces_a</c> a list of faces provide by a.
        /// </summary>
		public List<Face> faces_a;

        /// <summary>
        /// Instance variable <c>face_from_a</c> a number of faces provide by a.
        /// </summary>
        private int face_from_a = 0;

        /// <summary>
        /// Instance variable <c>faces_a</c> a list of faces provide by b.
        /// </summary>
		public List<Face> faces_b;
        

        /// <summary>
        /// Instance variable <c>face_from_b</c> a number of faces provide by b.
        /// </summary>
        private int face_from_b = 0;

        /// <summary>
        /// Instance variable <c>vertex_snap</c> represents the tolerance used.
        /// </summary>
		public float vertex_snap = 0.0f;

        /// <summary>
        /// Instance variable <c>snap_cache</c> represents a cache for the indice to add some faces.
        /// </summary>
		Dictionary<Vector3, int> snap_cache;

        /// <summary>
        /// Instance variable <c>scale_a</c> represents the scale of the brush A used in the operation.
        /// </summary>
		public Vector3 scale_a;

        /// <summary>
        /// This class is a comparer to sort a list with an id and a FaceBVH along the axis X
        /// </summary>
        public class FaceBVHCmpX : IComparer {
			int IComparer.Compare(object obj1, object obj2) {
                (int i,FaceBVH f) p_left = ((int ,FaceBVH)) obj1;
                (int i,FaceBVH f) p_right = ((int ,FaceBVH)) obj2;
				if(p_left.f.center.x == p_right.f.center.x){
                    return 0;
                }
				if(p_left.f.center.x < p_right.f.center.x){
                    return 1;
                }
                return -1;
			}
		};

        /// <summary>
        /// This class is a comparer to sort a list with an id and a FaceBVH along the axis Y
        /// </summary>
        public class FaceBVHCmpY : IComparer {
			int IComparer.Compare(object obj1, object obj2) {
                (int i,FaceBVH f) p_left = ((int ,FaceBVH)) obj1;
                (int i,FaceBVH f) p_right = ((int ,FaceBVH)) obj2;
				if(p_left.f.center.y == p_right.f.center.y){
                    return 0;
                }
				if(p_left.f.center.y < p_right.f.center.y){
                    return 1;
                }
                return -1;
			}
		};

        /// <summary>
        /// This class is a comparer to sort a list with an id and a FaceBVH along the axis Z
        /// </summary>
        public class FaceBVHCmpZ : IComparer {
			int IComparer.Compare(object obj1, object obj2) {
                (int i,FaceBVH f) p_left = ((int ,FaceBVH)) obj1;
                (int i,FaceBVH f) p_right = ((int ,FaceBVH)) obj2;
				if(p_left.f.center.z == p_right.f.center.z){
                    return 0;
                }
				if(p_left.f.center.z < p_right.f.center.z){
                    return 1;
                }
                return -1;
			}
		};

        /// <summary>
        /// This constructor initializes a new MeshMerge.
        /// </summary>
        public MeshMerge(int size_a = 0, int size_b = 0){
            points = new List<Vector3>();
            faces_a = new List<Face>(size_a);
            faces_b = new List<Face>(size_b);
            snap_cache = new Dictionary<Vector3, int>(3*(size_a+size_b));
        }

        /// <summary>
        /// This method is a recursive method to create a BVH representation of a mesh.
        /// </summary>
        /// <param><c>facebvh</c> an array with all faceBVH.</param>
        /// <param><c>id_facebvh</c> an array composed with a tupple with the id of the face BVH in the array facebvh, and the FaceBVH.</param>
        /// <param><c>from</c> the id where the id_facebvh start .</param>
        /// <param><c>size</c> the size of id_facebvh.</param>
        /// <param><c>depth</c> the depth for the FaceBVH.</param>
        /// <param><c>r_max_depth</c> the max depth for the FaceBVH.</param>
        /// <returns>
        /// Return the size of facebvh.
        /// </returns>
        int create_bvh(ref FaceBVH[] facebvh, ref (int i,FaceBVH f)[] id_facebvh, int from, int size, int depth, ref int r_max_depth) {
            if (depth > r_max_depth) {
                r_max_depth = depth;
            }

            if (size == 0) {
                return -1;
            }

            if (size <= BVH_LIMIT) {
                for (int i = 0; i < size - 1; i++) {
                    facebvh[id_facebvh[from + i].i].next = id_facebvh[from + i + 1].i;
                }
                return id_facebvh[from].i;
            }

            AABB aabb;
            aabb = new AABB(facebvh[id_facebvh[from].i].aabb.get_position(),facebvh[id_facebvh[from].i].aabb.get_size());
            for (int i = 1; i < size; i++) {
                aabb.merge_with(id_facebvh[from + i].f.aabb);
            }

            int li = aabb.get_longest_axis_index();

            switch (li) {
                case 0: {
                    SortArray temp = new SortArray(new FaceBVHCmpX());
                    temp.nth_element(from, from + size, from + size/2, ref id_facebvh);
                } break;

                case 1: {
                    SortArray temp = new SortArray(new FaceBVHCmpY());
                    temp.nth_element(from, from + size, from + size/2, ref id_facebvh);
                } break;

                case 2: {
                    SortArray temp = new SortArray(new FaceBVHCmpZ());
                    temp.nth_element(from, from + size, from + size/2, ref id_facebvh);
                } break;
            }

            int left = create_bvh(ref facebvh, ref id_facebvh, from, size / 2, depth + 1, ref r_max_depth);
            int right = create_bvh(ref facebvh, ref id_facebvh, from + size / 2, size - size / 2, depth + 1, ref r_max_depth);

            Array.Resize(ref facebvh, facebvh.Length + 1);
            facebvh[facebvh.Length - 1].aabb = aabb;
            facebvh[facebvh.Length - 1].center = aabb.get_center();
            facebvh[facebvh.Length - 1].face = -1;
            facebvh[facebvh.Length - 1].left = left;
            facebvh[facebvh.Length - 1].right = right;
            facebvh[facebvh.Length - 1].next = -1;

            return facebvh.Length - 1;
        }

        /// <summary>
        /// This method add a distance to r_intersectionsA or r_intersectionsB.
        /// </summary>
        /// <param><c>r_intersectionsA</c> a list of distance intersection.</param>
        /// <param><c>r_intersectionsB</c> a list of distance intersection.</param>
        /// <param><c>from_B</c> Bool coresponding if the intersections from B it true else false.</param>
        /// <param><c>p_distance</c> the distance you want add.</param>
        void add_distance(ref List<float> r_intersectionsA, ref List<float> r_intersectionsB, bool from_B, float distance) {
            List<float> intersections = from_B ? r_intersectionsB : r_intersectionsA;

            // Check if distance exists.
            foreach (float E in intersections) {
                if (Mathf.Abs(E - distance)<CSGBrush.CMP_EPSILON) {
                    return;
                }
            }

            intersections.Add(distance);
        }

        /// <summary>
        /// This method check if a face is in the facebvh.
        /// </summary>
        /// <param><c>facebvh</c> an array with all faceBVH.</param>
        /// <param><c>max_depth</c> the max depth of the array.</param>
        /// <param><c>bvh_first</c> the first id of the array .</param>
        /// <param><c>face_idx</c> the id of the face.</param>
        /// <param><c>from_faces_a</c> true if the face_idx is for the object a, false if is for the object b.</param>
        /// <returns>
        /// Return if the face is inside.
        /// </returns>
        bool bvh_inside(ref FaceBVH[] facebvh, int max_depth, int bvh_first, int face_idx, bool from_faces_a) {
            Face face;
            if(from_faces_a){
                face = faces_a[face_idx];
            }
            else{
                face = faces_b[face_idx];
            }
            
            Vector3[] face_points = {
                points[face.points[0]],
                points[face.points[1]],
                points[face.points[2]]
            };
            Vector3 face_center = (face_points[0] + face_points[1] + face_points[2]) / 3.0f;
            Vector3 face_normal = new PlaneCSG(face_points[0], face_points[1], face_points[2]).normal;

            int[] stack =  new int[max_depth];
 

            List<float> intersectionsA = new List<float>();
            List<float> intersectionsB = new List<float>();

            int level = 0;
            int pos = bvh_first;
            stack[0] = pos;
            int c = stack[level];

            while (true) {
                int node = stack[level] & (int)VISIT.NODE_IDX_MASK;
                FaceBVH? current_facebvh = facebvh[node];
                bool done = false;

                switch (stack[level] >> (int)VISIT.VISITED_BIT_SHIFT) {
                    case (int)VISIT.TEST_AABB_BIT: {
                        if (((FaceBVH)current_facebvh).face >= 0) {
                            while (current_facebvh!=null) {
                                if ( ((FaceBVH)current_facebvh).aabb.intersects_ray(face_center, face_normal)) {
                                    Face current_face;
                                    if(from_faces_a){
                                        current_face = faces_b[((FaceBVH)current_facebvh).face];
                                    }
                                    else{
                                        current_face = faces_a[((FaceBVH)current_facebvh).face];
                                    }
                                    Vector3[] current_points = {
                                        points[current_face.points[0]],
                                        points[current_face.points[1]],
                                        points[current_face.points[2]]
                                    };
                                    Vector3 current_normal = new PlaneCSG(current_points[0], current_points[1], current_points[2]).normal;
                                    Vector3 intersection_point = new Vector3();

                                    // Check if faces are co-planar.
                                    if (CSGBrush.is_equal_approx(current_normal, face_normal) &&
                                            CSGBrush.is_point_in_triangle(face_center, current_points)) {
                                        // Only add an intersection if not a B face.
                                        if (!face.from_b) {
                                            add_distance(ref intersectionsA, ref intersectionsB, current_face.from_b, 0);
                                        }
                                    } else if (CSGBrush.ray_intersects_triangle(face_center, face_normal, current_points, CSGBrush.CMP_EPSILON,ref intersection_point)) {
                                        float distance = Vector3.Distance(face_center, intersection_point);
                                        add_distance(ref intersectionsA, ref intersectionsB, current_face.from_b, distance);
                                    }
                                }

                                if (((FaceBVH)current_facebvh).next != -1) {
                                    current_facebvh = facebvh[((FaceBVH)current_facebvh).next];
                                } else {
                                    current_facebvh = null;
                                }
                            }

                            stack[level] = ((int)VISIT.VISIT_DONE_BIT << (int)VISIT.VISITED_BIT_SHIFT) | node;

                        } else {
                            bool valid = ((FaceBVH)current_facebvh).aabb.intersects_ray(face_center, face_normal);

                            if (!valid) {
                                stack[level] = ((int)VISIT.VISIT_DONE_BIT << (int)VISIT.VISITED_BIT_SHIFT) | node;
                            } else {
                                stack[level] = ((int)VISIT.VISIT_LEFT_BIT << (int)VISIT.VISITED_BIT_SHIFT) | node;
                            }
                        }
                        continue;
                    }

                    case (int)VISIT.VISIT_LEFT_BIT: {
                        stack[level] = ((int)VISIT.VISIT_RIGHT_BIT << (int)VISIT.VISITED_BIT_SHIFT) | node;
                        stack[level + 1] = ((FaceBVH)current_facebvh).left | (int)VISIT.TEST_AABB_BIT;
                        level++;
                        continue;
                    }

                    case (int)VISIT.VISIT_RIGHT_BIT: {
                        stack[level] = ((int)VISIT.VISIT_DONE_BIT << (int)VISIT.VISITED_BIT_SHIFT) | node;
                        stack[level + 1] = ((FaceBVH)current_facebvh).right | (int)VISIT.TEST_AABB_BIT;
                        level++;
                        continue;
                    }

                    case (int)VISIT.VISIT_DONE_BIT: {
                        if (level == 0) {
                            done = true;
                            break;
                        } else {
                            level--;
                        }
                        continue;
                    }
                }

                if (done) {
                    break;
                }
            }
            // Inside if face normal intersects other faces an odd number of times.
            int res = (intersectionsA.Count + intersectionsB.Count) & 1;
            return res != 0;
        }

        /// <summary>
        /// This method set up the face to kown if there are inside
        /// </summary>
        public void do_operation(Operation operation, ref CSGBrush r_merged_brush) {

            FaceBVH[] bvhvec_a = new FaceBVH[0];
            Array.Resize(ref bvhvec_a , faces_a.Count);
            FaceBVH[] facebvh_a = bvhvec_a;

            FaceBVH[] bvhvec_b = new FaceBVH[0];
            Array.Resize(ref bvhvec_b , faces_b.Count);
            FaceBVH[] facebvh_b = bvhvec_b;

            AABB aabb_a = new AABB();
            AABB aabb_b = new AABB();

            bool first_a = true;
            bool first_b = true;

            for (int i = 0; i < faces_a.Count; i++) {
                facebvh_a[i] = new FaceBVH();
                facebvh_a[i].left = -1;
                facebvh_a[i].right = -1;
                facebvh_a[i].face = i;
                facebvh_a[i].aabb = new AABB();
                facebvh_a[i].aabb.set_position(points[faces_a[i].points[0]]);
                facebvh_a[i].aabb.expand_to(points[faces_a[i].points[1]]);
                facebvh_a[i].aabb.expand_to(points[faces_a[i].points[2]]);
                facebvh_a[i].center = facebvh_a[i].aabb.get_center();
                facebvh_a[i].aabb.grow_by(vertex_snap);
                facebvh_a[i].next = -1;
                if (first_a) {
                    aabb_a = facebvh_a[i].aabb.Copy();
                    first_a = false;
                } else {
                    aabb_a.merge_with(facebvh_a[i].aabb);
                }
            }

            for (int i = 0; i < faces_b.Count; i++) {
                facebvh_b[i] = new FaceBVH();
                facebvh_b[i].left = -1;
                facebvh_b[i].right = -1;
                facebvh_b[i].face = i;
                facebvh_b[i].aabb = new AABB();
                facebvh_b[i].aabb.set_position(points[faces_b[i].points[0]]);
                facebvh_b[i].aabb.expand_to(points[faces_b[i].points[1]]);
                facebvh_b[i].aabb.expand_to(points[faces_b[i].points[2]]);
                facebvh_b[i].center = facebvh_b[i].aabb.get_center();
                facebvh_b[i].aabb.grow_by(vertex_snap);
                facebvh_b[i].next = -1;
                if (first_b) {
                    aabb_b = facebvh_b[i].aabb.Copy();
                    first_b = false;
                } else {
                    aabb_b.merge_with(facebvh_b[i].aabb);
                }
            }

            AABB intersection_aabb = aabb_a.intersection(aabb_b);
            
            // Check if shape AABBs intersect.
            if (operation==Operation.OPERATION_INTERSECTION && intersection_aabb.get_size() == Vector3.zero) {
                //return;
            }

            (int,FaceBVH)[] bvhtrvec_a = new (int,FaceBVH)[0];
            Array.Resize(ref bvhtrvec_a, faces_a.Count);
            (int,FaceBVH)[] bvhptr_a = bvhtrvec_a;
            for (int i = 0; i < faces_a.Count; i++) {
                bvhptr_a[i] = (i , facebvh_a[i]);
            }

            (int,FaceBVH)[] bvhtrvec_b = new (int,FaceBVH)[0];
            Array.Resize(ref bvhtrvec_b, faces_b.Count);
            (int,FaceBVH)[] bvhptr_b = bvhtrvec_b;
            for (int i = 0; i < faces_b.Count; i++) {
                bvhptr_b[i] = (i , facebvh_b[i]);
            }

            int max_depth_a = 0;
            create_bvh(ref facebvh_a, ref bvhptr_a, 0, face_from_a, 1, ref max_depth_a);
            int max_alloc_a = facebvh_a.Length;

            int max_depth_b = 0;
            create_bvh(ref facebvh_b, ref bvhptr_b, 0, face_from_b, 1, ref max_depth_b);
            int max_alloc_b = facebvh_b.Length;

            switch (operation) {
                case Operation.OPERATION_UNION: {
                    int faces_count = 0;
                    Array.Resize(ref r_merged_brush.faces, faces_a.Count+faces_b.Count);
                    
                    for (int i = 0; i < faces_a.Count; i++) {
                        // Check if face AABB intersects the intersection AABB.
                        if (!intersection_aabb.intersects_inclusive(facebvh_a[i].aabb)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_a[i].uvs[0],faces_a[i].uvs[1],faces_a[i].uvs[2]};
                            faces_count++;
                            continue;
                        }

                        if (!bvh_inside(ref facebvh_b, max_depth_b, max_alloc_b - 1, i, true)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_a[i].uvs[0],faces_a[i].uvs[1],faces_a[i].uvs[2]};
                            faces_count++;
                        }
                    }

                    for (int i = 0; i < faces_b.Count; i++) {
                        // Check if face AABB intersects the intersection AABB.
                        if (!intersection_aabb.intersects_inclusive(facebvh_b[i].aabb)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_b[i].uvs[0],faces_b[i].uvs[1],faces_b[i].uvs[2]};
                            faces_count++;
                            continue;
                        }

                        if (!bvh_inside(ref facebvh_a, max_depth_a, max_alloc_a - 1, i, false)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_b[i].uvs[0],faces_b[i].uvs[1],faces_b[i].uvs[2]};
                            faces_count++;
                        }
                    }
                    Array.Resize(ref r_merged_brush.faces, faces_count);

                } break;

                case Operation.OPERATION_INTERSECTION: {
                    int faces_count = 0;
                    Array.Resize(ref r_merged_brush.faces, faces_a.Count+faces_b.Count);
                    
                    for (int i = 0; i < faces_a.Count; i++) {
                        // Check if face AABB intersects the intersection AABB.
                        if (!intersection_aabb.intersects_inclusive(facebvh_a[i].aabb)) {
                            continue;
                        }

                        if (bvh_inside(ref facebvh_b, max_depth_b, max_alloc_b - 1, i, true)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_a[i].uvs[0],faces_a[i].uvs[1],faces_a[i].uvs[2]};
                            faces_count++;
                        }
                    }

                    for (int i = 0; i < faces_b.Count; i++) {
                        // Check if face AABB intersects the intersection AABB.
                        if (!intersection_aabb.intersects_inclusive(facebvh_b[i].aabb)) {
                            continue;
                        }

                        if (bvh_inside(ref facebvh_a, max_depth_a, max_alloc_a - 1, i, false)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_b[i].uvs[0],faces_b[i].uvs[1],faces_b[i].uvs[2]};
                            faces_count++;
                        }
                    }
                    Array.Resize(ref r_merged_brush.faces, faces_count);
                } break;

                case Operation.OPERATION_SUBTRACTION: {
                    int faces_count = 0;
                    Array.Resize(ref r_merged_brush.faces, faces_a.Count+faces_b.Count);
                    
                    for (int i = 0; i < faces_a.Count; i++) {
                        // Check if face AABB intersects the intersection AABB.
                        if (!intersection_aabb.intersects_inclusive(facebvh_a[i].aabb)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_a[i].uvs[0],faces_a[i].uvs[1],faces_a[i].uvs[2]};
                            faces_count++;
                            continue;
                        }

                        if (!bvh_inside(ref facebvh_b, max_depth_b, max_alloc_b - 1, i, true)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_a[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_a[i].uvs[0],faces_a[i].uvs[1],faces_a[i].uvs[2]};
                            faces_count++;
                        }
                    }

                    for (int i = 0; i < faces_b.Count; i++) {
                        // Check if face AABB intersects the intersection AABB.
                        if (!intersection_aabb.intersects_inclusive(facebvh_b[i].aabb)) {
                            continue;
                        }

                        if (bvh_inside(ref facebvh_a, max_depth_a, max_alloc_a - 1, i, false)) {
                            r_merged_brush.faces[faces_count].vertices = new List<Vector3>(3);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[1]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[0]]);
                            r_merged_brush.faces[faces_count].vertices.Add(points[faces_b[i].points[2]]);
                            r_merged_brush.faces[faces_count].uvs = new Vector2[3]{faces_b[i].uvs[0],faces_b[i].uvs[1],faces_b[i].uvs[2]};
                            faces_count++;
                        }
                    }
                    Array.Resize(ref r_merged_brush.faces, faces_count);

                } break;
            }
        }

        /// <summary>
        /// This method add face in the meshMerge.
        /// </summary>
        /// <param><c>points</c> an array with the vertices of the face.</param>
        /// <param><c>uvs</c> an array with the array for each points.</param>
        /// <param><c>from_b</c> if the face come from the face B.</param>
        public void add_face( Vector3[] points, Vector2[] uvs, bool from_b) {
            int[] indices = new int[3];
            for (int i = 0; i < 3; i++) {
                Vector3 vk = new Vector3();
                vk.x = Mathf.RoundToInt( ((points[i].x + vertex_snap) * 0.31234f) / vertex_snap);
                vk.y = Mathf.RoundToInt( ((points[i].y + vertex_snap) * 0.31234f) / vertex_snap);
                vk.z = Mathf.RoundToInt( ((points[i].z + vertex_snap) * 0.31234f) / vertex_snap);

                if (snap_cache.ContainsKey(vk)) {
                    indices[i] = snap_cache[vk];
                } else {
                    indices[i] = this.points.Count;
                    this.points.Add(Vector3.Scale(points[i],scale_a));
                    snap_cache.Add(vk, indices[i]);
                }
            }

            // Don't add degenerate faces.
            if (indices[0] == indices[2] || indices[0] == indices[1] || indices[1] == indices[2]) {
                return;
            }

            Face face = new Face();
            face.from_b = from_b;
            face.points = new int[3];
            face.uvs = new Vector2[3];

            for (int k = 0; k < 3; k++) {
                face.points[k] = indices[k];
                face.uvs[k] = uvs[k];
            }
            if(from_b){
                face_from_b++;
                faces_b.Add(face);
            }
            else{
                face_from_a++;
                faces_a.Add(face);
            }

        }

    }
}

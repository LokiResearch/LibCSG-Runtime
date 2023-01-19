using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace LibCSG{

/**
* <summary>
* Class <c>CSGBrush</c> is a class represents a brush to the CSG with the geometry of the object you want use to do CSG operation.
* </summary>
**/
public class CSGBrush
{
    /// <summary>
    /// Instance variable <c>obj</c> is the GameObject link if there is one.
    /// </summary>
    public GameObject obj;
    
    /// <summary>
    /// Instance static variable <c>CMP_EPSILON</c> use like a tolerance for some function.
    /// </summary>
    public static float CMP_EPSILON = 0.000001f;

    /// <summary>
    /// Instance structure <c>Face</c> represents a face of the object give in the brush.
    /// </summary>
    public struct Face {
		public List<Vector3> vertices;
		public Vector2[] uvs;
		public AABB aabb;
	};

    /// <summary>
    /// Instance variable <c>faces</c> represents all face of the object.
    /// </summary>
    public Face[] faces;

    /// <summary>
    /// This static method check is the squared distance between of the two points is more little than the squared distance give in parameters
    /// </summary>
    /// <param><c>point1</c> a point.</param>
    /// <param><c>point2</c> an other point.</param>
    /// <param><c>distance</c> the distance you want compare.</param>
    /// <returns>
    /// Return True if the squared distance between of the two points is more little than the squared distance else False 
    /// </returns>
    public static bool is_snapable(Vector3 point1, Vector3 point2, float distance) {
        return (point1 - point2).sqrMagnitude < distance * distance;
    }

    /// <summary>
    /// This static method check is two vector are equal with a tolerance corresponding to the CMP_EPSILON
    /// </summary>
    /// <param><c>vec1</c> a Vector3 you want compare to another.</param>
    /// <param><c>vec2</c> a Vector3 you want compare to another.</param>
    /// <returns>
    /// Return True if the two vector are approximately equal else False 
    /// </returns>
    public static bool is_equal_approx(Vector3 vec1, Vector3 vec2){
        Vector3 vec3 = vec1-vec2;
        return Mathf.Abs(vec3.x) < CMP_EPSILON && Mathf.Abs(vec3.y) < CMP_EPSILON && Mathf.Abs(vec3.z) < CMP_EPSILON;
    }

    /// <summary>
    /// This static method check is two vector are equal with a tolerance corresponding to the CMP_EPSILON
    /// </summary>
    /// <param><c>vec1</c> a Vector2 you want compare to another.</param>
    /// <param><c>vec2</c> a Vector2 you want compare to another.</param>
    /// <returns>
    /// Return True if the two vector are approximately equal else False 
    /// </returns>
    public static bool is_equal_approx(Vector2 vec1, Vector2 vec2){
        Vector3 vec3 = vec1-vec2;
        return Mathf.Abs(vec3.x) < CMP_EPSILON && Mathf.Abs(vec3.y) < CMP_EPSILON;
    }

    /// <summary>
    /// This static method calculate an uv (Vector2) corresponding to the linear interpolation for the interpolate point in a segment
    /// </summary>
    /// <param><c>segment_points</c> an array with 2 vector2 corresponding to the first and last point of the segment.</param>
    /// <param><c>uvs</c> an array with 2 vector2 corresponding to the uv to the first and last point of the segment.</param>
    /// <param><c>interpolation</c> the point you want calculate the interpolate uv.</param>
    /// <returns>
    /// Return the uv to the interpolate point
    /// </returns>
    public static Vector2 interpolate_segment_uv(Vector2[] segment_points, Vector2[] uvs, Vector2 interpolation) {
        if (CSGBrush.is_equal_approx(segment_points[0], segment_points[1])) {
            return uvs[0];
	    }

        float segment_length = Vector2.Distance(segment_points[0], segment_points[1]);
        float distance = Vector2.Distance(segment_points[0], interpolation);
        float fraction = distance / segment_length;

        return Vector2.Lerp(uvs[0], uvs[1], fraction);
    }

    /// <summary>
    /// This static method calculate an uv (Vector2) corresponding to the interpolate point in a triangle
    /// </summary>
    /// <param><c>segment_points</c> an array with 3 vector2 corresponding to 3 point of the triangle.</param>
    /// <param><c>uvs</c> an array with 3 vector2 corresponding to 3 point of the triangle.</param>
    /// <param><c>interpolation_point</c> the point you want calculate the interpolate uv.</param>
    /// <returns>
    /// Return the uv to the interpolate point.
    /// </returns>
    public static Vector2 interpolate_triangle_uv(Vector2[] vertices, Vector2[] uvs, Vector2 interpolation_point) {
        if (CSGBrush.is_equal_approx(interpolation_point, vertices[0])) {
            return uvs[0];
        }
        if (CSGBrush.is_equal_approx(interpolation_point, vertices[1])) {
            return uvs[1];
        }
        if (CSGBrush.is_equal_approx(interpolation_point, vertices[2])) {
            return uvs[2];
        }

        Vector2 edge1 = vertices[1] - vertices[0];
        Vector2 edge2 = vertices[2] - vertices[0];
        Vector2 interpolation = interpolation_point - vertices[0];

        float edge1_on_edge1 = Vector2.Dot(edge1, edge1);
        float edge1_on_edge2 = Vector2.Dot(edge1, edge2);
        float edge2_on_edge2 = Vector2.Dot(edge2, edge2);
        float inter_on_edge1 = Vector2.Dot(interpolation, edge1);
        float inter_on_edge2 = Vector2.Dot(interpolation, edge2);
        float scale = (edge1_on_edge1 * edge2_on_edge2 - edge1_on_edge2 * edge1_on_edge2);
        if (scale == 0) {
            return uvs[0];
        }

        float v = (edge2_on_edge2 * inter_on_edge1 - edge1_on_edge2 * inter_on_edge2) / scale;
        float w = (edge1_on_edge1 * inter_on_edge2 - edge1_on_edge2 * inter_on_edge1) / scale;
        float u = 1.0f - v - w;

        return uvs[0] * u + uvs[1] * v + uvs[2] * w;
    }

    /// <summary>
    /// This static method check if a ray intersect the triangle and if it is true, this methode calculate the intersection point
    /// </summary>
    /// <param><c>from</c> the point where the ray start.</param>
    /// <param><c>dir</c> the direction of the ray.</param>
    /// <param><c>vertices</c> an array with 3 vector3 corresponding to 3 point of the triangle.</param>
    /// <param><c>tolerance</c> is a tolerance you want used.</param>
    /// <param><c>intersection_point</c> a Vector3 to set the intersection point between the ray and the triangle.</param>
    /// <returns>
    /// Return True if the ray intersects the triangle and put the intersection point else return False
    /// </returns>
    public static bool ray_intersects_triangle(Vector3 from, Vector3 dir, Vector3[] vertices, float tolerance, ref Vector3 intersection_point) {
        Vector3 edge1 = vertices[1] - vertices[0];
        Vector3 edge2 = vertices[2] - vertices[0];
        Vector3 h = Vector3.Cross(dir, edge2);
        float a = Vector3.Dot(edge1, h);
        // Check if ray is parallel to triangle.
        if (Mathf.Abs(a) < CMP_EPSILON ) {
            return false;
        }
        float f = 1.0f / a;

        Vector3 s = from - vertices[0];
        float u = f * Vector3.Dot(s, h);
        if (u < 0.0 - tolerance || u > 1.0 + tolerance) {
            return false;
        }

        Vector3 q = Vector3.Cross(s, edge1);
        float v = f * Vector3.Dot(dir, q);
        if (v < 0.0 - tolerance || u + v > 1.0 + tolerance) {
            return false;
        }

        // Ray intersects triangle.
        // Calculate distance.
        float t = f * Vector3.Dot(edge2, q);
        // Confirm triangle is in front of ray.
        if (t >= tolerance) {
            intersection_point = from + dir * t;
            return true;
        } else {
            return false;
        }
    }

    /// <summary>
    /// This static method check if the point is in the triangle.
    /// </summary>
    /// <param><c>point</c> the point you want know if it is in the triangle.</param>
    /// <param><c>vertices</c> an arrays with 3 points corresponding to the triangle.</param>
    /// <returns>
    /// Return True if the point is in the triangle else return False
    /// </returns>
    public static bool is_point_in_triangle(Vector3 point, Vector3[] vertices, int shifted = 0) {
        float det = Vector3.Dot(vertices[0], Vector3.Cross(vertices[1],vertices[2]));

        // If determinant is, zero try shift the triangle and the point.
        if (Mathf.Abs(det) < CMP_EPSILON ) {
            if (shifted > 2) {
                // Triangle appears degenerate, so ignore it.
                return false;
            }
            Vector3 shift_by = Vector3.zero;
            shift_by[shifted] = 1;
            Vector3 shifted_point = point + shift_by;
            Vector3[] shifted_vertices = {vertices[0] + shift_by, vertices[1] + shift_by, vertices[2] + shift_by };
            return is_point_in_triangle(shifted_point, shifted_vertices, shifted + 1);
        }

        // Find the barycentric coordinates of the point with respect to the vertices.
        float[] lambda = new float[3];
        lambda[0] = Vector3.Dot(point, Vector3.Cross(vertices[1], vertices[2])) / det;
        lambda[1] = Vector3.Dot(point, Vector3.Cross(vertices[2], vertices[0])) / det;
        lambda[2] = Vector3.Dot(point, Vector3.Cross(vertices[0], vertices[1])) / det;

        // Point is in the plane if all lambdas sum to 1.
        if (!(Mathf.Abs((lambda[0] + lambda[1] + lambda[2]) - 1) < CMP_EPSILON)) {
            return false;
        }

        // Point is inside the triangle if all lambdas are positive.
        if (lambda[0] < 0 || lambda[1] < 0 || lambda[2] < 0) {
            return false;
        }

        return true;
    }

    /// <summary>
    /// This static method check if the triangle is degenerate.
    /// </summary>
    /// <param><c>vertices</c> an arrays with 3 points corresponding to the triangle.</param>
    /// <param><c>tolerance</c> is a tolerance you want used.</param>
    /// <returns>
    /// Return True if the triangle is degenerate else False.
    /// </returns>
    public static bool is_triangle_degenerate(Vector2[] vertices, float tolerance) {
        float det = vertices[0].x * vertices[1].y - vertices[0].x * vertices[2].y +
                vertices[0].y * vertices[2].x - vertices[0].y * vertices[1].x +
                vertices[1].x * vertices[2].y - vertices[1].y * vertices[2].x;

        return det < tolerance;
    }

    /// <summary>
    /// This static method check if the segments are parallel given in the parameters.
    /// </summary>
    /// <param><c>segment1_points</c> an array with 2 vector2 corresponding to the first and last point of the segment.</param>
    /// <param><c>segment2_points</c> an array with 2 vector2 corresponding to the first and last point of the segment.</param>
    /// <param><c>tolerance</c> is a tolerance you want used.</param>
    /// <returns>
    /// Return True if the segments are parallel else False.
    /// </returns>
    public static bool are_segments_parallel(Vector2[] segment1_points, Vector2[] segment2_points, float tolerance) {
        Vector2 segment1 = segment1_points[1] - segment1_points[0];
        Vector2 segment2 = segment2_points[1] - segment2_points[0];
        float segment1_length2 = Vector3.Dot(segment1, segment1);
        float segment2_length2 = Vector3.Dot(segment2, segment2);
        float segment_onto_segment = Vector3.Dot(segment2, segment1);

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
    /// This constructor initializes a new CSGBrush.
    /// <param><c>name</c> a name for the GameObject create with the brush.</param>
    /// </summary>
    public CSGBrush(string name=""){ 
        faces = new Face[0];
        obj = new GameObject();
        if(name != ""){
            obj.name = name;
        }
    }

    /// <summary>
    /// This constructor initializes a new CSGBrush link to a GameObject.
    /// </summary>
    /// <param><c>objet</c> is a GameObject you want link with the brush.</param>
    public CSGBrush(GameObject objet){ 
        faces = new Face[0];
        obj = objet;
    }

    /// <summary>
    /// This method build the all faces of the CSGBrush with a list of vertices and a list of uv. We supposed the vertices n , n+1, n+2 in the list is atriangle of the mesh
    /// </summary>
    /// <param><c>vertices</c> is a List of Vector3 corresponding to the vertices.</param>
    /// <param><c>uvs</c> is a List of Vector2 corresponding to the uvs.</param>
    public void build_from_faces(List<Vector3> vertices, List<Vector2> uvs) {
        Array.Clear(faces,0,faces.Length);
        List<Vector3> rv = vertices;
        List<Vector2> ruv = uvs;

        Array.Resize(ref faces,vertices.Count / 3);

        for (int i = 0; i < faces.Length; i++) {
            Face new_face = new Face();
            new_face.vertices = new List<Vector3>(3);
            new_face.vertices.Add(vertices[i * 3 + 2]);
            new_face.vertices.Add(vertices[i * 3 + 1]);
            new_face.vertices.Add(vertices[i * 3 + 0]);
            new_face.uvs = new Vector2[3];
            new_face.uvs[0] = ruv[i * 3 + 2];
            new_face.uvs[1] = ruv[i * 3 + 1];
            new_face.uvs[2] = ruv[i * 3 + 0];

            faces[i] = new_face;
        }

        regen_face_aabbs();
    }

    /// <summary>
    /// This method build the all faces of the CSGBrush with a mesh
    /// </summary>
    /// <param><c>mesh</c> the mesh you want used.</param>
    public void build_from_mesh(Mesh mesh) {
        Array.Clear(faces,0,faces.Length);

        Array.Resize(ref faces,mesh.triangles.Length / 3);

        for (int i = 0; i < faces.Length; i++) {
            Face new_face = new Face();
            new_face.vertices = new List<Vector3>(3);
            new_face.vertices.Add(mesh.vertices[mesh.triangles[i * 3 + 2]]);
            new_face.vertices.Add(mesh.vertices[mesh.triangles[i * 3 + 1]]);
            new_face.vertices.Add(mesh.vertices[mesh.triangles[i * 3 + 0]]);
            if(mesh.uv.Length!=0){
                new_face.uvs = new Vector2[3]{mesh.uv[mesh.triangles[i * 3 + 2]], mesh.uv[mesh.triangles[i * 3 + 1]], mesh.uv[mesh.triangles[i * 3 + 0]]};
            }
            else{
                new_face.uvs = new Vector2[3]{new Vector2(), new Vector2(), new Vector2()};
            }

            faces[i] = new_face;
        }
    }

    /// <summary>
    /// This method build the AABB for each face of the brush
    /// </summary>
    public void regen_face_aabbs() {
        for (int i = 0; i < faces.Length; i++) {
            faces[i].aabb = new AABB();
            faces[i].aabb.set_position(obj.transform.TransformPoint(faces[i].vertices[0]));
            faces[i].aabb.expand_to(obj.transform.TransformPoint(faces[i].vertices[1]));
            faces[i].aabb.expand_to(obj.transform.TransformPoint(faces[i].vertices[2]));
        }
    }

    /// <summary>
    /// This method get the mesh used in the Brush.
    /// </summary>
    /// <param><c>m</c> If you don't want create another mesh you can give your mesh.</param>
    /// <returns>
    /// Return the Mesh used in the Brush.
    /// </returns>
    public Mesh getMesh(Mesh m = null){
        if(m==null){
            m = new Mesh();
        }
        Vector3[] vert = new Vector3[faces.Length*3];
        Vector2[] uv = new Vector2[faces.Length*3];
		int[] triangles = new int[faces.Length*3];
        for (int i = 0; i < faces.Length; i++) {
            vert[3 * i + 2] = faces[i].vertices[0];
            vert[3 * i + 1] = faces[i].vertices[1];
            vert[3 * i] = faces[i].vertices[2];
            triangles[3 * i] = 3 * i;
            triangles[3 * i + 1] = 3 * i + 1;
            triangles[3 * i + 2] = 3 * i + 2;
            uv[3 * i + 2] = faces[i].uvs[0];
            uv[3 * i + 1] = faces[i].uvs[1];
            uv[3 * i] = faces[i].uvs[2];
        }
        m.vertices = vert;
        m.triangles = triangles;
        m.uv = uv;
        m.RecalculateNormals();
        return m;
    }

}
}

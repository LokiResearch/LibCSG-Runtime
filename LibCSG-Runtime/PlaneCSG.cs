using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace LibCSG{
    /**
    * <summary>
    * Class <c>PlaneCSG</c> is a class represents a plane.
    * </summary>
    **/
    public class PlaneCSG
    {
        /// <summary>
        /// Instance variable <c>normal</c> represents the normal of the plane.
        /// </summary>
        public Vector3 normal;

        /// <summary>
        /// Instance variable <c>d</c> is the distance measured from the Plane to the origin, along the Plane's normal.
        /// </summary>
        public float d = 0;

        /// <summary>
        /// This constructor initializes the new Plane with 3 points
        /// </summary>
        /// <param><c>point1</c> a point on the plane.</param>
        /// <param><c>point2</c> a point on the plane.</param>
        /// <param><c>point3</c> a point on the plane.</param>
        public PlaneCSG( Vector3 point1,  Vector3 point2, Vector3 point3) {
            normal = Vector3.Cross((point1 - point3), (point1 - point2));
            normal /= normal.magnitude;
            d = Vector3.Dot(normal, point1);
        }

        /// <summary>
        /// This method see if the point given in parameters is over the plane
        /// </summary>
        /// <param><c>point</c> is a Point you know if it is over the plane.</param>
        /// <returns>
        /// Return True if the points is over the plane else False 
        /// </returns>
        public bool is_point_over(Vector3 point) {
            return (Vector3.Dot(normal, point) > d);
        }

        /// <summary>
        /// This method see if the point given in parameters is on the plan (you can give a tolerance)
        /// </summary>
        /// <param><c>point</c> is a Point you know if it is on the plane.</param>
        /// <param><c>tolerance</c> is a tolerance you want used.</param>
        /// <returns>
        /// Return True if the points is on the plane else False 
        /// </returns>
        public bool has_point(Vector3 point, float tolerance = 0) {
            float dist = Vector3.Dot(normal, point) - d;
            dist = Mathf.Abs(dist);
            return (dist <= tolerance);
        }
    }
}
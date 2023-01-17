using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace LibCSG{
/**
* <summary>
* Class <c>AABB</c> is a bounding box use to create a bvh of the mesh.
* </summary>
**/
public class AABB
{
    /// <summary>
    /// Instance variable <c>size</c> represents the size of the box.
    /// </summary>
    public float sizex;
    public float sizey;
    public float sizez;

    /// <summary>
    /// Instance variable <c>position</c> represents the position of the box (the bottom left corner not the center).
    /// </summary>
    public float positionx;
    public float positiony;
    public float positionz;

    /// <summary>
    /// This constructor initializes the position to (0,0,0) and the size to (0,0,0).
    /// </summary>
    public AABB(){
        positionx = 0;
        positiony = 0;
        positionz = 0;
        sizex = 0;
        sizey = 0;
        sizez = 0;
    }

    /// <summary>
    /// This constructor initializes the new Box to
    /// (<paramref name="pos"/>,<paramref name="size"/>).
    /// </summary>
    /// <param><c>pos</c> is the new position of the box.</param>
    /// <param><c>size</c> is the new size of the box.</param>
    public AABB(Vector3 pos, Vector3 size){
        this.positionx = pos.x;
        this.positiony = pos.y;
        this.positionz = pos.z;
        this.sizex = size.x;
        this.sizey = size.y;
        this.sizez = size.z;
    }

    /// <summary>
    /// This method expand the Box to include the new Point give
    /// </summary>
    /// <param><c>point</c> is a Point you want include in the box.</param>
    public void expand_to(Vector3 point) {
        float endx = positionx + sizex;
        float endy = positiony + sizey;
        float endz = positionz + sizez;

        if (point.x < this.positionx) {
            this.positionx = point.x;
        }
        if (point.y < this.positiony) {
            this.positiony = point.y;
        }
        if (point.z < this.positionz) {
            this.positionz = point.z;
        }

        if (point.x > endx) {
            endx = point.x;
        }
        if (point.y > endy) {
            endy = point.y;
        }
        if (point.z > endz) {
            endz = point.z;
        }

        this.sizex = endx - this.positionx;
        this.sizey = endy - this.positiony;
        this.sizez = endz - this.positionz;
    }

    /// <summary>
    /// This method see if the box given in parameter intersects this box
    /// </summary>
    /// <returns>
    /// Return True if the box given in parameter intersects this box else False 
    /// </returns>
    public bool intersects_inclusive(AABB aabb) {
        return !((positionx > (aabb.positionx + aabb.sizex)) || ((positionx + sizex) < aabb.positionx) || (positiony > (aabb.positiony + aabb.sizey)) || ((positiony + sizey) < aabb.positiony) || (positionz > (aabb.positionz + aabb.sizez)) || ((positionz + sizez) < aabb.positionz));
    }

    /// <summary>
    /// This method get the center of the box
    /// </summary>
    /// <returns>
    /// Return <c> Vector3</c> corresponding to the center of the Box
    /// </returns>
    public Vector3 get_center(){
		return new Vector3(positionx + (sizex * 0.5f), positiony + (sizey * 0.5f), positionz + (sizez * 0.5f));
    }

    /// <summary>
    /// This method get the position of the box
    /// </summary>
    /// <returns>
    /// Return <c> Vector3</c> corresponding to the position of the Box
    /// </returns>
    public Vector3 get_position(){
		return new Vector3(positionx,positiony,positionz);
    }

    /// <summary>
    /// This method set the position of the box
    /// </summary>
    /// <param><c>position</c> the new position of the AABB.</param>
    public void set_position(Vector3 position){
        this.positionx = position.x;
        this.positiony = position.y;
        this.positionz = position.z;
    }

    /// <summary>
    /// This method get the size of the box
    /// </summary>
    /// <returns>
    /// Return <c> Vector3</c> corresponding to the size of the Box
    /// </returns>
    public Vector3 get_size(){
		return new Vector3(sizex,sizey,sizez);
    }

    /// <summary>
    /// This method increase the box in each direction by the float given in parameters
    /// </summary>
    /// <param><c>size_grow</c> the size you want to increase.</param>
    public void grow_by(float size_grow) {
        this.positionx -= size_grow;
        this.positiony -= size_grow;
        this.positionz -= size_grow;
        this.sizex += 2.0f * size_grow;
        this.sizey += 2.0f * size_grow;
        this.sizez += 2.0f * size_grow;
    }

    /// <summary>
    /// This method merge the Box given in parameter with this box
    /// </summary>
    /// <param><c>aabb</c> the box you want merge.</param>
    public void merge_with(AABB aabb) {
        float minx;
        float miny;
        float minz;

        minx = (positionx < aabb.positionx) ? positionx : aabb.positionx;
        miny = (positiony < aabb.positiony) ? positiony : aabb.positiony;
        minz = (positionz < aabb.positionz) ? positionz : aabb.positionz;

        this.sizex = (sizex + positionx > aabb.sizex + aabb.positionx) ? (sizex + positionx) - minx : (aabb.sizex + aabb.positionx) - minx;
        this.sizey = (sizey + positiony > aabb.sizey + aabb.positiony) ? (sizey + positiony) - miny : (aabb.sizey + aabb.positiony) - miny;
        this.sizez = (sizez + positionz > aabb.sizez + aabb.positionz) ? (sizez + positionz) - minz : (aabb.sizez + aabb.positionz) - minz;

        this.positionx = minx;
        this.positiony = miny;
        this.positionz = minz;
    }

    /// <summary>
    /// This method get a copy of this box
    /// </summary>
    /// <returns>
    /// Return <c>AABB</c> corresponding to the copy of this AABB.
    /// </returns>
    public AABB Copy(){
        return new AABB( new Vector3(positionx,positiony,positionz),  new Vector3(sizex,sizey,sizez));
    }

    /// <summary>
    /// This method get the intersection between this box and the box given in parameter
    /// </summary>
    /// <returns>
    /// Return <c>AABB</c> corresponding to the intersection between this box and the box given in parameter.
    /// </returns>
    public AABB intersection(AABB aabb) {
        float src_maxx = positionx + sizex;
        float src_maxy = positiony + sizey;
        float src_maxz = positionz + sizez;
        float dst_minx = aabb.positionx;
        float dst_miny = aabb.positiony;
        float dst_minz = aabb.positionz;
        float dst_maxx = aabb.positionx + aabb.sizex;
        float dst_maxy = aabb.positiony + aabb.sizey;
        float dst_maxz = aabb.positionz + aabb.sizez;

        Vector3 min, max;

        if ((positionx > dst_maxx || src_maxx < dst_minx) || (positiony > dst_maxy || src_maxy < dst_miny) || (positionz > dst_maxz || src_maxz < dst_minz)){
            return new AABB();
        }
        
        min.x = (positionx > dst_minx) ? positionx : dst_minx;
        max.x = (src_maxx < dst_maxx) ? src_maxx : dst_maxx;

        min.y = (positiony > dst_miny) ? positiony : dst_miny;
        max.y = (src_maxy < dst_maxy) ? src_maxy : dst_maxy;

        min.z = (positionz > dst_minz) ? positionz : dst_minz;
        max.z = (src_maxz < dst_maxz) ? src_maxz : dst_maxz;

        return new AABB(min, max - min);
    }

    /// <summary>
    /// This method get the index of the longest axis for this box
    /// </summary>
    /// <returns>
    /// Return <c>int</c> corresponding to the longest axis. 0 if it is X, 1 if it is Y and 2 if it is Z.
    /// </returns>
    public int get_longest_axis_index() {
        int axis = 0;
        float max_size = sizex;

        if (sizey > max_size) {
            axis = 1;
            max_size = sizey;
        }

        if (sizez > max_size) {
            axis = 2;
        }

        return axis;
    }

    /// <summary>
    /// This method check if the ray create by a position and a direction intersects the AABB
    /// </summary>    
    /// <param><c>from</c> the position where the ray start.</param>
    /// <param><c>dir</c> the direction of the ray.</param>
    /// <returns>
    /// Return True if the ray create by a position and a direction intersects the AABB else false.
    /// </returns>
    public bool intersects_ray(Vector3 from, Vector3 dir)  {
        Vector3 c1 = Vector3.zero;
        Vector3 c2 = Vector3.zero;
        Vector3 position = new Vector3(positionx,positiony,positionz);
        Vector3 size = new Vector3(sizex,sizey,sizez);
        Vector3 end = position + size;
        float near = -1e20f;
        float far = 1e20f;
        int axis = 0;

        for (int i = 0; i < 3; i++) {
            if (dir[i] == 0) {
                if ((from[i] < position[i]) || (from[i] > end[i])) {
                    return false;
                }
            } else { 
                c1[i] = (position[i] - from[i]) / dir[i];
                c2[i] = (end[i] - from[i]) / dir[i];

                if (c1[i] > c2[i]) {
                    Vector3 temp = c1;
                    c1 = c2;
                    c2 = temp;
                }
                if (c1[i] > near) {
                    near = c1[i];
                    axis = i;
                }
                if (c2[i] < far) {
                    far = c2[i];
                }
                if ((near > far) || (far < 0)) {
                    return false;
                }
            }
        }

        return true;
    }
}
}
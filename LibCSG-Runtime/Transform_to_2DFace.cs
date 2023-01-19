using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace LibCSG{

/**
* <summary>
* Class <c>Transform_to_2DFace</c> is a class used in build2DFaces to transform a 3D point into a 2D point.
* </summary>
**/
public class Transform_to_2DFace
{

    /// <summary>
    /// Instance variable <c>Basisl1</c>,<c>Basisl2</c>,<c>Basisl3</c> represents the line of the 3×3 matrix used for 3D rotation and scale.
    /// </summary>
    private Vector3 Basisl1 = new Vector3(1, 0, 0);
    private Vector3 Basisl2 = new Vector3(0, 1, 0);
    private Vector3 Basisl3 = new Vector3(0, 0, 1);

    /// <summary>
    /// Instance variable <c>position</c> represents the position of the transform.
    /// </summary>
    private Vector3 position = new Vector3(0,0,0);
    
    /// <summary>
    /// This constructor initializes the position to (0,0,0) and the Basis to an identity matrix.
    /// </summary>
    public Transform_to_2DFace(){
    }

    /// <summary>
    /// This constructor initializes the new Transform to
    /// (<paramref name="pos"/>,<paramref name="basisl1"/><paramref name="basisl2"/><paramref name="basisl3"/>).
    /// </summary>
    /// <param><c>pos</c> is the new position of the transform.</param>
    /// <param><c>basisl1</c> is the new basisl1 of the transform.</param>
    /// <param><c>basisl2</c> is the new basisl2 of the transform.</param>
    /// <param><c>basisl3</c> is the new basisl3 of the transform.</param>
    public Transform_to_2DFace(Vector3 pos, Vector3 basisl1, Vector3 basisl2, Vector3 basisl3){
        this.Basisl1 = basisl1;
        this.Basisl2 = basisl2;
        this.Basisl3 = basisl3;
        this.position = pos;
    }

    /// <summary>
    /// This method set the column <paramref name="col"/> 
    /// </summary>
    /// <param><c>col</c> the number of the column you want change.</param>
    /// <param><c>valeur</c> the new number you want set in the column.</param>
    public void basis_set_column(int col, Vector3 valeur){
        this.Basisl1[col] = valeur[0];
        this.Basisl2[col] = valeur[1];
        this.Basisl3[col] = valeur[2];
    }

    /// <summary>
    /// This method get the column <paramref name="col"/> of the basis
    /// </summary>
    /// <param><c>col</c> the number of the column you want get.</param>
    /// <returns>
    /// Return <c> Vector3</c> corresponding to the column <paramref name="col"/> of the basis
    /// </returns>
    public Vector3 basis_get_column(int col){
        return new Vector3(Basisl1[col], Basisl2[col], Basisl3[col]);
    }

    /// <summary>
    /// This method transformed the Point by the transform.
    /// </summary>
    /// <param><c>p_vector</c> the Point you want transform.</param>
    /// <returns>
    /// Return <c>Vector3</c> corresponding to the transformed point
    /// </returns>
    public Vector3 xform(Vector3 vector)  {
	return new Vector3(
			Vector3.Dot(Basisl1, vector) + position.x,
			Vector3.Dot(Basisl2 ,vector) + position.y,
			Vector3.Dot(Basisl3 ,vector) + position.z);
    }

  
    /// <summary>
    /// This method transformed the Point by the basis matrix.
    /// </summary>
    /// <param><c>p_vector</c> the Point you want transform.</param> 
    /// <returns>
    /// Return <c>Vector3</c> corresponding to the transformed point
    /// </returns> 
    private Vector3 Basis_xform(Vector3 vector) {
        return new Vector3(
                Vector3.Dot(Basisl1, vector),
                Vector3.Dot(Basisl2, vector),
                Vector3.Dot(Basisl3, vector));
    }

    /// <summary>
    /// This method invert the transform
    /// </summary>
    public void affine_invert() {
        this.Basis_invert();
        this.position = this.Basis_xform(-position);
    }

    /// <summary>
    /// This method invert the Basis
    /// </summary>
    private void Basis_invert() {
        float[] co = {
            cofac(ref Basisl2, 1, ref Basisl3, 2), cofac(ref Basisl2, 2, ref Basisl3, 0), cofac(ref Basisl2, 0, ref Basisl3, 1)
        };
        float det = Basisl1[0] * co[0] + Basisl1[1] * co[1] + Basisl1[2] * co[2];

        float s = 1.0f / det;

        this.Set_basis(co[0] * s, cofac(ref Basisl1, 2, ref Basisl3, 1) * s, cofac(ref Basisl1, 1, ref Basisl2, 2) * s,
                co[1] * s, cofac(ref Basisl1, 0, ref Basisl3, 2) * s, cofac(ref Basisl1, 2, ref Basisl2, 0) * s,
                co[2] * s, cofac(ref Basisl1, 1, ref Basisl3, 0) * s, cofac(ref Basisl1, 0, ref Basisl2, 1) * s);
    }

    /// <summary>
    /// This method calculate the co-factor of the basis matrix
    /// <param><c>row1</c> a line.</param>
    /// <param><c>col1</c> the index of column.</param>
    /// <param><c>row2</c> another line.</param>
    /// <param><c>col2</c> another the index of column.</param>
    /// </summary>
    private float cofac(ref Vector3 row1, int col1, ref Vector3 row2, int col2) {
	    return row1[col1] * row2[col2] - row1[col2] * row2[col1];
    }

    /// <summary>
    /// This method set the basis
    /// </summary>
    /// <param><c>xx</c> the new value of Basis[0][0].</param>
    /// <param><c>xy</c> the new value of Basis[0][1].</param>
    /// <param><c>xz</c> the new value of Basis[0][2].</param>
    /// <param><c>yx</c> the new value of Basis[1][0].</param>
    /// <param><c>yy</c> the new value of Basis[1][1].</param>
    /// <param><c>yz</c> the new value of Basis[1][2].</param>
    /// <param><c>zx</c> the new value of Basis[2][0].</param>
    /// <param><c>zy</c> the new value of Basis[2][1].</param>
    /// <param><c>zz</c> the new value of Basis[2][2].</param>
    public void Set_basis(float xx, float xy, float xz, float yx, float yy, float yz, float zx, float zy, float zz) {
		this.Basisl1[0] = xx;
		this.Basisl1[1] = xy;
		this.Basisl1[2] = xz;
		this.Basisl2[0] = yx;
		this.Basisl2[1] = yy;
		this.Basisl2[2] = yz;
		this.Basisl3[0] = zx;
		this.Basisl3[1] = zy;
		this.Basisl3[2] = zz;
	}

    /// <summary>
    /// This method set the position of the transform
    /// </summary>
    /// <param><c>pos</c> the new position of the transform.</param>
    public void Set_position(Vector3 pos) {
        this.position.Set(pos.x, pos.y, pos.z);
    }

    /// <summary>
    /// This method return new transform corresponding to the invert this transform.
    /// </summary>
    public Transform_to_2DFace affine_inverse() {
        Transform_to_2DFace res = new Transform_to_2DFace(this.position, Basisl1, Basisl2, Basisl3);
        res.affine_invert();
        return res;
    }
}
}
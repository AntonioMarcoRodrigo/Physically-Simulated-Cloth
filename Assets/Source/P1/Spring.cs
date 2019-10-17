using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring : MonoBehaviour {

    #region InEditorVariables

    public float Stiffness;
    public float Damping;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public enum SpringType { Stretch, Bend };
    public SpringType springType;

    public float Length0;
    public float Length;
    public Vector3 dir;

    private PhysicsManager Manager;

    public Spring(Node a, Node b, SpringType s)
    {
        nodeA = a;
        nodeB = b;
        springType = s;
    }

    // Update is called once per frame
    void Update () {
	}

    // Use this for initialization
    public void Initialize(float stiffness, float damping, PhysicsManager m)
    {
        // TO BE COMPLETED
        Stiffness = stiffness;
        Damping = damping;
        Manager = m;

        UpdateState();
        Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        dir = nodeA.Pos - nodeB.Pos;
        Length = dir.magnitude;
        dir = (1.0f / Length) * dir;
    }

    // Compute force and add to the nodes
    public void ComputeForce()
    {
        // TO BE COMPLETED
        Vector3 Force = -Stiffness * (Length - Length0) * (nodeA.Pos - nodeB.Pos) / Length;
        nodeA.Force += Force;
        nodeB.Force -= Force;
    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
        MatrixXD u = MatrixXD.Build.Dense(3, 1);
        MatrixXD uTrasp = MatrixXD.Build.Dense(1, 3);
        MatrixXD identity = MatrixXD.Build.Dense(3, 3);
        MatrixXD dudx = MatrixXD.Build.Dense(3, 1);

        MatrixXD dFadXa = MatrixXD.Build.Dense(3, 3);
        MatrixXD dFbdXa = MatrixXD.Build.Dense(3, 3);
        MatrixXD dFadXb = MatrixXD.Build.Dense(3, 3);
        MatrixXD dFbdXb = MatrixXD.Build.Dense(3, 3);

        MatrixXD dFadVa = MatrixXD.Build.Dense(3, 3);
        MatrixXD dFbdVa = MatrixXD.Build.Dense(3, 3);
        MatrixXD dFadVb = MatrixXD.Build.Dense(3, 3);
        MatrixXD dFbdVb = MatrixXD.Build.Dense(3, 3);

        int A = nodeA.index;
        int B = nodeB.index;

        u[0, 0] = (nodeA.Pos.x - nodeB.Pos.x) / Length;
        u[1, 0] = (nodeA.Pos.y - nodeB.Pos.y) / Length;
        u[2, 0] = (nodeA.Pos.z - nodeB.Pos.z) / Length;

        uTrasp[0, 0] = u[0, 0];
        uTrasp[0, 1] = u[1, 0];
        uTrasp[0, 2] = u[2, 0];

        identity[0, 0] = 1;
        identity[1, 0] = 0;
        identity[2, 0] = 0;

        identity[0, 1] = 0;
        identity[1, 1] = 1;
        identity[2, 1] = 0;

        identity[0, 2] = 0;
        identity[1, 2] = 0;
        identity[2, 2] = 1;

        dudx = 1 / Length * (identity - (u * uTrasp));

        dFadXa = -Stiffness * ((Length - Length0) * dudx + u * uTrasp);
        dFbdXb = dFadXa;
        dFbdXa = -dFadXa;
        dFadXb = -dFbdXb;

        dFdx.SetSubMatrix(A, A, dFdx.SubMatrix(A, 3, A, 3) + dFadXa);
        dFdx.SetSubMatrix(A, B, dFdx.SubMatrix(B, 3, B, 3) + dFadXb);
        dFdx.SetSubMatrix(B, A, dFdx.SubMatrix(B, 3, A, 3) + dFbdXa);
        dFdx.SetSubMatrix(B, B, dFdx.SubMatrix(B, 3, B, 3) + dFbdXb);

        dFadVa = -Damping * (u * uTrasp);
        dFbdVb = -dFadVa;
        dFbdVa = -dFadVa;
        dFadVb = -dFbdVb;

        dFdv.SetSubMatrix(A, A, dFdv.SubMatrix(A, 3, A, 3) + dFadVa);
        dFdv.SetSubMatrix(A, B, dFdv.SubMatrix(B, 3, B, 3) + dFadVb);
        dFdv.SetSubMatrix(B, A, dFdv.SubMatrix(B, 3, A, 3) + dFbdVa);
        dFdv.SetSubMatrix(B, B, dFdv.SubMatrix(B, 3, B, 3) + dFbdVb);
    }

}

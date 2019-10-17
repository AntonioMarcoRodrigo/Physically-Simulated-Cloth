using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic mass-spring model component which can be dropped onto
/// a game object and configured so that the set of nodes and
/// edges behave as a mass-spring model.
/// </summary>
public class MassSpring : MonoBehaviour, ISimulable
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public MassSpring()
    {
        Manager = null;
    }

    #region EditorVariables

    public List<Node> Nodes;
    public List<Spring> Springs;

    public float Mass;
    public float StiffnessStretch;
    public float StiffnessBend;
    public float DampingAlpha;
    public float DampingBeta;

    #endregion

    #region OtherVariables
    private PhysicsManager Manager;

    private int index;
    #endregion

    #region MonoBehaviour

    public void Awake()
    {
        Mesh mesh = this.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // TO BE COMPLETED
        for (int i=0; i<mesh.vertexCount; i++)
        {
            Node node = new Node (vertices[i]);
            Nodes.Insert(i, node);
        }

        for (int i = 0; i < triangles.Length; i += 3)
        {          
            Spring springAB = new Spring(Nodes[triangles[i]], Nodes[triangles[i + 1]], Spring.SpringType.Stretch);           
            Spring springBC = new Spring(Nodes[triangles[i + 1]], Nodes[triangles[i + 2]], Spring.SpringType.Stretch);       
            Spring springCA = new Spring(Nodes[triangles[i + 2]], Nodes[triangles[i]], Spring.SpringType.Stretch);

            bool AB = false;
            bool BC = false;
            bool CA = false;

            for (int j=0; j<Springs.Count; j++)
            {
                if ((System.Object.ReferenceEquals(Springs[j].nodeA, springAB.nodeA) && System.Object.ReferenceEquals(Springs[j].nodeB, springAB.nodeB)) || 
                   (System.Object.ReferenceEquals(Springs[j].nodeA, springAB.nodeB) && System.Object.ReferenceEquals(Springs[j].nodeB, springAB.nodeA)))
                {                   
                    AB = true;                    
                }

                if ((System.Object.ReferenceEquals(Springs[j].nodeA, springBC.nodeA) && System.Object.ReferenceEquals(Springs[j].nodeB, springBC.nodeB)) ||
                   (System.Object.ReferenceEquals(Springs[j].nodeA, springBC.nodeB) && System.Object.ReferenceEquals(Springs[j].nodeB, springBC.nodeA)))
                {
                    BC = true;
                }

                if ((System.Object.ReferenceEquals(Springs[j].nodeA, springCA.nodeA) && System.Object.ReferenceEquals(Springs[j].nodeB, springCA.nodeB)) ||
                   (System.Object.ReferenceEquals(Springs[j].nodeA, springCA.nodeB) && System.Object.ReferenceEquals(Springs[j].nodeB, springCA.nodeA)))
                {
                    CA = true;
                }
            }

            if (AB == false)
                Springs.Add(springAB);
            if (BC == false)
                Springs.Add(springBC);
            if (CA == false)
                Springs.Add(springCA);
        }        
        
    }

    public void Update()
    {
        // TO BE COMPLETED
        Mesh mesh = this.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        for (int i=0; i<Nodes.Count; i++)
            vertices[i] = Nodes[i].Pos;

        mesh.vertices = vertices;
    }

    public void FixedUpdate()
    {
        // TO BE COMPLETED
    }
    #endregion

    #region ISimulable

    public void Initialize(int ind, PhysicsManager m, List<Fixer> fixers)
    {
        // TO BE COMPLETED
        Manager = m;

        index = ind;

        // Start scene nodes/edges
        for (int i = 0; i < Nodes.Count; ++i)
        {
            Nodes[i].Initialize(index + 3 * i, Mass, DampingAlpha, Manager); // Prepare
            for (int j = 0; j < fixers.Count; j++)
            {
                if (fixers[j].IsInside(Nodes[i].Pos))
                    Nodes[i].Fixed = true;
            }
        }

        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].Initialize(StiffnessStretch, DampingBeta, Manager); // Prepare
    }

    public int GetNumDoFs()
    {
        return 3 * Nodes.Count;
    }

    public void GetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetPosition(position);
    }

    public void SetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetPosition(position);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].UpdateState();
    }

    public void GetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetVelocity(velocity);
    }

    public void SetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetVelocity(velocity);
    }

    public void GetForce(VectorXD force)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForce(force);
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForceJacobian(dFdx, dFdv);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForceJacobian(dFdx, dFdv);
    }

    public void GetMass(MatrixXD mass)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMass(mass);
    }

    public void GetMassInverse(MatrixXD massInv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMassInverse(massInv);
    }

    public void ComputeForces()
    {
        // TO BE COMPLETED
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].ResetForce();
            Nodes[i].ComputeForce();
        }

        for (int i = 0; i < Springs.Count; i++)
        {
            Springs[i].ComputeForce();
        }
    }

    public void FixVector(VectorXD v)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixVector(v);
        }
    }

    public void FixMatrix(MatrixXD M)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixMatrix(M);
        }
    }

    #endregion

    #region OtherMethods

    #endregion

}

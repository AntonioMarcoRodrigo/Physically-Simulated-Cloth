using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		Paused = true;
		TimeStep = 0.01f;
		Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		IntegrationMethod = Integration.Explicit;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		Symplectic = 1,
        Midpoint = 2,
        Verlet = 3,
        Implicit = 4,
	};

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
    public List<GameObject> SimObjects;
    public List<Fixer> Fixers;
    public Integration IntegrationMethod;

    #endregion

    #region OtherVariables
    private List<ISimulable> m_objs;
    private int m_numDoFs;
    public bool first;
    public VectorXD xOld;
    #endregion

    #region MonoBehaviour

    public void Start()
    {
 
        //Parse the simulable objects and initialize their state indices
        m_numDoFs = 0;
        m_objs = new List<ISimulable>(SimObjects.Capacity);

        foreach (GameObject obj in SimObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();
            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable object
                simobj.Initialize(m_numDoFs, this, Fixers);

                // Retrieve pos and vel size
                m_numDoFs += simobj.GetNumDoFs();
            }
        }

        first = true;

    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;

    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Select integration method
        switch (this.IntegrationMethod)
        {
            case Integration.Explicit: this.stepExplicit(); break;
            case Integration.Symplectic: this.stepSymplectic(); break;
            case Integration.Midpoint: this.stepMidpoint(); break;
            case Integration.Verlet: this.stepVerlet(); break;
            case Integration.Implicit: this.stepImplicit(); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step using Explicit integration.
    /// </summary>
    private void stepExplicit()
	{
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.ComputeForces();
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x += TimeStep * v;
        v += TimeStep * (Minv * f);

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Symplectic integration.
    /// </summary>
    private void stepSymplectic()
	{
        // TO BE COMPLETED
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.ComputeForces();
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        v += TimeStep * (Minv * f);
        x += TimeStep * v;

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Midpoint integration.
    /// </summary>
    private void stepMidpoint()
    {
        // TO BE COMPLETED
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.ComputeForces();
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        VectorXD x0 = x;
        VectorXD v0 = v;

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x += 0.5f * TimeStep * v;
        v += 0.5f * TimeStep * (Minv * f);

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v); 
            obj.ComputeForces();
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x = x0 + TimeStep * v;
        v = v0 + TimeStep * (Minv * f);

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Verlet integration.
    /// </summary>
    private void stepVerlet()
    {
        // TO BE COMPLETED
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.ComputeForces();
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        VectorXD x0 = x;

        if (first)
        {
            x += TimeStep * v + 0.5f * TimeStep * TimeStep * (Minv * f);
            first = false;
        }
        else
        {
            x = 2 * x - xOld + TimeStep * TimeStep * (Minv * f);
        }

        v = (1.0f / TimeStep) * (x - x0);

        xOld = x0;

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
        // TO BE COMPLETED
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD M = new DenseMatrixXD(m_numDoFs);
        MatrixXD dFx = new DenseMatrixXD(m_numDoFs);
        MatrixXD dFv = new DenseMatrixXD(m_numDoFs);

        MatrixXD A = new DenseMatrixXD(m_numDoFs);
        VectorXD b = new DenseVectorXD(m_numDoFs);

        M.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.ComputeForces();
            obj.GetForce(f);
            obj.GetMass(M);
            obj.GetForceJacobian(dFx, dFv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(M);
        }

        A = M + TimeStep * -dFv + TimeStep * TimeStep * -dFx;
        b = M * v + TimeStep * f;

        v = A.Solve(b);
        x += TimeStep * v;

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

}

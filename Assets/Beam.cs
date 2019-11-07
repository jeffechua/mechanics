using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;

public interface Force
{
	float GetLowerEdge();
	float GetUpperEdge();
	float GetPressureAt(float point, float resolution); // Force per unit length (N/m)
	float GetShearAt(float point);    // -∫ P dx (N)
	float GetMomentAt(float point);   //  ∫ S dx (N m)
}

public class PointForce : Force
{
	public float point; // ∈ [0,L] in m
	public float force; // net force
	public PointForce(float point, float force)
	{
		this.point = point;
		this.force = force;
	}
	public float GetLowerEdge() => point;
	public float GetUpperEdge() => point;
	public float GetPressureAt(float point, float resolution)
		=> Mathf.Abs(point - this.point) < resolution / 2 ? force/resolution : 0;
	public float GetShearAt(float point) => point > this.point ? -force : 0;
	public float GetMomentAt(float point) => point > this.point ? -force * (point - this.point) : 0;
}

[Serializable]
public class PolynomialForce : Force {
	public float ledge; // ∈ [0,L] in m
	public float uedge; // ∈ [0,L] in m
	public float[] coefficients; // coefficients to polynomial that evaluates pressure (in N/m) at a point (in m)
	public PolynomialForce (float ledge, float uedge, params float[] coefficients) {
		this.ledge = ledge;
		this.uedge = uedge;
		this.coefficients = coefficients;
	}
	public float GetLowerEdge() => ledge;
	public float GetUpperEdge() => uedge;
	public float GetPressureAt(float point, float resolution) {
		if (point < ledge || point > uedge)
			return 0;
		float sum = 0;
		float x = point - ledge;
		for(int i=0; i<coefficients.Length; i++)
			sum += coefficients[i] * Mathf.Pow(x, i);
		return sum;
	}
	public float GetShearAt(float point) {
		if (point < ledge)
			return 0;
		float sum = 0;
		float x = point > uedge ? uedge : (point - ledge);
		for (int i = 0; i < coefficients.Length; i++)
			sum -= coefficients[i] * Mathf.Pow(x, i+1) / (i+1);
		return sum;
	}
	public float GetMomentAt(float point) {
		if (point < ledge)
			return 0;
		float sum = 0;
		float x = point > uedge ? uedge : (point - ledge);
		for (int i = 0; i < coefficients.Length; i++)
			sum -= coefficients[i] * Mathf.Pow(x, i + 2) / (i + 1) / (i + 2);
		if (point > uedge) {
			sum += GetShearAt(uedge) * (point - uedge);
		}
		return sum;
	}
}

public class Beam : MonoBehaviour
{

	Vector2 origin;
	float length;

	public GameObject particlesTemplate;
	ParticleSystem particles;
	ParticleSystem.EmitParams ep;

	public GameObject lineTemplate;
	public int samples = 20;
	LineRenderer pressureLine;
	LineRenderer shearLine;
	LineRenderer momentLine;

	public float[] pressure;
	public float[] shear;
	public float[] moment;
	public List<Force> forces = new List<Force>();

	// Start is called before the first frame update
	void Start() {
		length = transform.localScale.x;

		particles = Instantiate(particlesTemplate, transform.position, transform.rotation, transform).GetComponent<ParticleSystem>();
		ep = new ParticleSystem.EmitParams();
		ep.startColor = Color.red;
		ep.startSize = 0.2f;

		pressureLine = Instantiate(lineTemplate, transform.position, transform.rotation, transform).GetComponent<LineRenderer>();
		pressureLine.startColor = Color.red;
		pressureLine.endColor = Color.red;
		pressureLine.positionCount = samples;

		shearLine = Instantiate(lineTemplate, transform.position, transform.rotation, transform).GetComponent<LineRenderer>();
		shearLine.startColor = Color.green;
		shearLine.endColor = Color.green;
		shearLine.positionCount = samples;

		momentLine = Instantiate(lineTemplate, transform.position, transform.rotation, transform).GetComponent<LineRenderer>();
		momentLine.startColor = Color.blue;
		momentLine.endColor = Color.blue;
		momentLine.positionCount = samples;
	}

    // Update is called once per frame
    void FixedUpdate()
    {
		forces.Clear();

		origin = transform.position - transform.right*length / 2;

		Rigidbody2D rb = GetComponent<Rigidbody2D>();
		// Self
		forces.Add(new PolynomialForce(0, length, rb.mass * rb.gravityScale * Vector2.Dot(Physics2D.gravity, transform.up) / length));

		// Contacts
		List<ContactPoint2D> contacts = new List<ContactPoint2D>();
		GetComponent<Collider2D>().GetContacts(contacts);
		particles.Clear();
		foreach (ContactPoint2D contact in contacts) {
			ep.position = contact.point;
			ep.startSize = contact.normalImpulse;
			particles.Emit(ep, 1);
			int sign = Vector2.Dot(contact.point - origin, transform.up) < 0 ? 1 : -1; // collision from below pushes up
			forces.Add(new PointForce(To1D(contact.point), sign * contact.normalImpulse / Time.fixedDeltaTime));
		}

		pressure = new float[samples];
		shear = new float[samples];
		moment = new float[samples];
		foreach(Force force in forces) {
			for(int i=0; i<samples; i++) {
				pressure[i] += force.GetPressureAt(length * i / (samples - 1), length/samples);
				shear[i] += force.GetShearAt(length * i / (samples - 1));
				moment[i] += force.GetMomentAt(length * i / (samples - 1));
			}
		}
		for (int i = 0; i < samples; i++) {
			pressureLine.SetPosition(i, ToWorldSpace(length * i / (samples - 1)) + (Vector2)transform.up * pressure[i] / 100);
			shearLine.SetPosition(i, ToWorldSpace(length * i / (samples - 1)) + (Vector2)transform.up * shear[i] / 20);
			momentLine.SetPosition(i, ToWorldSpace(length * i / (samples - 1)) + (Vector2)transform.up * moment[i] / 20);
		}
		
    }

	float To1D (Vector2 point) => Vector2.Dot(point - origin, transform.right);
	Vector2 ToWorldSpace(float point) => origin + (Vector2)transform.right * point;
}

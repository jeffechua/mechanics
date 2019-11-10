using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;

public interface Force {
	float GetLowerEdge();
	float GetUpperEdge();
	float GetPressureAt(float point, float resolution); // Force per unit length (N/m)
	float GetShearAt(float point);    // -∫ P dx (N)
	float GetMomentAt(float point);   //  ∫ S dx (N m)
}

public class PointForce : Force {
	public float point; // ∈ [0,L] in m
	public float force; // net force
	public PointForce(float point, float force) {
		this.point = point;
		this.force = force;
	}
	public float GetLowerEdge() => point;
	public float GetUpperEdge() => point;
	public float GetPressureAt(float point, float resolution)
		=> Mathf.Abs(point - this.point) < resolution / 2 ? force / resolution : 0;
	public float GetShearAt(float point) => point > this.point ? -force : 0;
	public float GetMomentAt(float point) => point > this.point ? -force * (point - this.point) : 0;
}

public class PointMoment : Force {
	public float point;
	public float moment;
	public PointMoment(float point, float moment) {
		this.point = point;
		this.moment = moment;
	}
	public float GetLowerEdge() => point;
	public float GetUpperEdge() => point;
	public float GetPressureAt(float point, float resolution) => 0;
	public float GetShearAt(float point) =>  0;
	public float GetMomentAt(float point) => point > this.point ? moment : 0;
}

[Serializable]
public class PolynomialForce : Force {
	public float ledge; // ∈ [0,L] in m
	public float uedge; // ∈ [0,L] in m
	public float[] coefficients; // coefficients to polynomial that evaluates pressure (in N/m) at a point (in m)
	public PolynomialForce(float ledge, float uedge, params float[] coefficients) {
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
		for (int i = 0; i < coefficients.Length; i++)
			sum += coefficients[i] * Mathf.Pow(x, i);
		return sum;
	}
	public float GetShearAt(float point) {
		if (point < ledge)
			return 0;
		float sum = 0;
		float x = point > uedge ? (uedge - ledge) : (point - ledge);
		for (int i = 0; i < coefficients.Length; i++)
			sum -= coefficients[i] * Mathf.Pow(x, i + 1) / (i + 1);
		return sum;
	}
	public float GetMomentAt(float point) {
		if (point < ledge)
			return 0;
		float sum = 0;
		float x = point > uedge ? (uedge - ledge) : (point - ledge);
		for (int i = 0; i < coefficients.Length; i++)
			sum -= coefficients[i] * Mathf.Pow(x, i + 2) / (i + 1) / (i + 2);
		if (point > uedge) {
			sum += GetShearAt(uedge) * (point - uedge);
		}
		return sum;
	}
}

public class PolynomialMoment : Force {
	public float ledge; // ∈ [0,L] in m
	public float uedge; // ∈ [0,L] in m
	public float[] coefficients; // coefficients to polynomial that evaluates moment (in Nm) at a point (in m)
	public PolynomialMoment(float ledge, float uedge, params float[] coefficients) {
		this.ledge = ledge;
		this.uedge = uedge;
		this.coefficients = coefficients;
	}
	public float GetLowerEdge() => ledge;
	public float GetUpperEdge() => uedge;
	public float GetPressureAt(float point, float resolution) => 0;
	public float GetShearAt(float point) => 0;
	public float GetMomentAt(float point) {
		if (point < ledge)
			return 0;
		float sum = 0;
		float x = point - ledge;
		for (int i = 0; i < coefficients.Length; i++)
			sum += coefficients[i] * Mathf.Pow(x, i);
		return sum;
	}
}

public class Beam : MonoBehaviour {

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
	void FixedUpdate() {

		forces.Clear();
		particles.Clear();

		origin = transform.position - transform.right * length / 2;

		Rigidbody2D rb = GetComponent<Rigidbody2D>();
		// Self
		forces.Add(new PolynomialForce(0, length, rb.mass * rb.gravityScale * Vector2.Dot(Physics2D.gravity, transform.up) / length));

		// Parse collider contacts
		List<ContactPoint2D> contacts = new List<ContactPoint2D>();
		GetComponent<Collider2D>().GetContacts(contacts);
		List<Tuple<ContactPoint2D, ContactPoint2D>> edges = new List<Tuple<ContactPoint2D, ContactPoint2D>>();
		int i = 0;
		while (i < contacts.Count) {
			int j = i + 1;
			while (j < contacts.Count) {
				if (contacts[i].collider == contacts[j].collider) {
					edges.Add(new Tuple<ContactPoint2D, ContactPoint2D>(contacts[i], contacts[j]));
					contacts.RemoveAt(j);
					contacts.RemoveAt(i);
					i--;
					break;
				}
				j++;
			}
			i++;
		}
		// Process "true" contact points
		foreach (ContactPoint2D contact in contacts) {

			ep.position = contact.point;
			ep.startColor = Color.red;
			ep.startSize = contact.normalImpulse;
			particles.Emit(ep, 1);

			ep.startColor = contact.tangentImpulse > 0 ? Color.blue : Color.green;
			ep.startSize = Mathf.Abs(contact.tangentImpulse);
			particles.Emit(ep, 1);

			forces.Add(new PointForce(To1D(contact.point), GetPerpendicularForce(contact)));
			forces.Add(new PointMoment(To1D(contact.point), GetParallelForce(contact) * transform.localScale.y / 2));
		}
		// Process detected contact edges
		foreach (Tuple<ContactPoint2D, ContactPoint2D> edge in edges) {
			float ledge, uedge, lforce, uforce;
			float edge1 = To1D(edge.Item1.point);
			float edge2 = To1D(edge.Item2.point);
			if (edge1 < edge2) {
				ledge = edge1;
				uedge = edge2;
				lforce = GetPerpendicularForce(edge.Item1);
				uforce = GetPerpendicularForce(edge.Item2);
			} else {
				ledge = edge2;
				uedge = edge1;
				lforce = GetPerpendicularForce(edge.Item2);
				uforce = GetPerpendicularForce(edge.Item1);
			}
			// Assume weight is distributed linearly, i.e. 1st order PolynomialForce
			float upressure = uforce / (uedge - ledge) * 2;
			float lpressure = lforce / (uedge - ledge) * 2;
			float gradient = (upressure - lpressure) / (uedge - ledge);
			forces.Add(new PolynomialForce(ledge, uedge, lpressure, gradient));
			// Model tangential force as uniform (not linear)
			float netMoment = GetParallelForce(edge.Item1) + GetParallelForce(edge.Item2);
			float mgradient = netMoment / (uedge - ledge);
			forces.Add(new PolynomialMoment(ledge, uedge, 0, mgradient * transform.localScale.y / 2));
		}

		pressure = new float[samples];
		shear = new float[samples];
		moment = new float[samples];
		foreach (Force force in forces) {
			for (i = 0; i < samples; i++) {
				pressure[i] += force.GetPressureAt(length * i / (samples - 1), length / samples);
				shear[i] += force.GetShearAt(length * i / (samples - 1));
				moment[i] += force.GetMomentAt(length * i / (samples - 1));
			}
		}
		for (i = 0; i < samples; i++) {
			pressureLine.SetPosition(i, ToWorldSpace(length * i / (samples - 1)) + (Vector2)transform.up * pressure[i] / 100);
			shearLine.SetPosition(i, ToWorldSpace(length * i / (samples - 1)) + (Vector2)transform.up * shear[i] / 20);
			momentLine.SetPosition(i, ToWorldSpace(length * i / (samples - 1)) + (Vector2)transform.up * moment[i] / 30);
		}

	}

	float GetPerpendicularForce (ContactPoint2D contact) {
		return contact.normalImpulse / Time.fixedDeltaTime * Mathf.Sign(Vector2.Dot(contact.normal, transform.up));
	}
	float GetParallelForce(ContactPoint2D contact) {
		return contact.tangentImpulse / Time.fixedDeltaTime;
	}
	float To1D(Vector2 point) => Vector2.Dot(point - origin, transform.right);
	Vector2 ToWorldSpace(float point) => origin + (Vector2)transform.right * point;
}

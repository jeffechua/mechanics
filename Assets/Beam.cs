using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;

public interface Force
{
	float GetLowerEdge();
	float GetUpperEdge();
	float GetPressureAt(float point);
	float GetShearAt(float point);
	float GetMomentAt(float point);
}

[Serializable]
public class PointForce : Force
{
	public float point;
	public float force;
	public PointForce(float point, float force)
	{
		this.point = point;
		this.force = force;
	}
	public float GetLowerEdge() => point;
	public float GetUpperEdge() => point;
	public float GetPressureAt(float point) => point == this.point ? force : 0;
	public float GetShearAt(float point) => point > this.point ? force : 0;
	public float GetMomentAt(float point) => point > this.point ? force * (point - this.point) : 0;
}

public class Beam : MonoBehaviour
{

	Vector2 origin;
	float length;

	public GameObject particlesTemplate;
	ParticleSystem particles;
	ParticleSystem.EmitParams ep;

	public GameObject lineTemplate;
	LineRenderer shearLine;
	LineRenderer momentLine;
	public int samples = 20;
	public List<PointForce> forces = new List<PointForce>();
	public float[] shear;
	public float[] moment;

	// Start is called before the first frame update
	void Start() {
		length = transform.localScale.x;

		particles = Instantiate(particlesTemplate, transform.position, transform.rotation, transform).GetComponent<ParticleSystem>();
		ep = new ParticleSystem.EmitParams();
		ep.startColor = Color.red;
		ep.startSize = 0.2f;

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
    void Update()
    {
		forces.Clear();

		origin = transform.position - transform.right*length / 2;
		List<ContactPoint2D> contacts = new List<ContactPoint2D>();
		GetComponent<Collider2D>().GetContacts(contacts);
		particles.Clear();
		foreach (ContactPoint2D contact in contacts) {
			ep.position = contact.point;
			ep.startSize = contact.normalImpulse;
			particles.Emit(ep, 1);
			int sign = Vector2.Dot(contact.point - origin, transform.up) > 0 ? 1 : -1;
			forces.Add(new PointForce(To1D(contact.point), sign * contact.normalImpulse));
		}

		shear = new float[samples];
		moment = new float[samples];
		foreach(Force force in forces) {
			for(int i=0; i<samples; i++) {
				shear[i] += force.GetShearAt((float)i / (samples - 1));
				moment[i] += force.GetMomentAt((float)i / (samples - 1));
			}
		}
		for(int i=0; i<samples; i++) {
			shearLine.SetPosition(i, ToWorldSpace((float)i / (samples - 1)) + (Vector2)transform.up * shear[i]);
			momentLine.SetPosition(i, ToWorldSpace((float)i / (samples - 1)) + (Vector2)transform.up * moment[i]);
		}
		
    }

	float To1D (Vector2 point) => Vector2.Dot(point - origin, transform.right)/length;
	Vector2 ToWorldSpace(float point) => origin + (Vector2)transform.right * point * length;
}

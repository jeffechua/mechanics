using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ContactShower : MonoBehaviour
{

	public GameObject particlesTemplate;
	ParticleSystem particles;
	ParticleSystem.EmitParams ep;

	// Start is called before the first frame update
	void Start() {
		particles = Instantiate(particlesTemplate, transform.position, transform.rotation, transform).GetComponent<ParticleSystem>();
		ep = new ParticleSystem.EmitParams();
		ep.startColor = Color.red;
		ep.startSize = 0.2f;
	}

    // Update is called once per frame
    void Update()
    {
		List<ContactPoint2D> contacts = new List<ContactPoint2D>();
		GetComponent<Collider2D>().GetContacts(contacts);
		particles.Clear();
		foreach (ContactPoint2D contact in contacts) {
			ep.position = contact.point;
			ep.startSize = contact.normalImpulse;
			particles.Emit(ep, 1);
		}
    }
}

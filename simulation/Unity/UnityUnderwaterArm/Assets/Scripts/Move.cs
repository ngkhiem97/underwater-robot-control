using UnityEngine;

public class Move : MonoBehaviour
{
	private Rigidbody rb;
	private Vector3 movingDirection;
	void Start()
	{
		rb = GetComponent<Rigidbody> ();
		float speed = 0.2f;
		float x = 0;
		float y = Random.Range(-speed, speed);
		float z = 0;
		movingDirection = new Vector3(x, y, z);
	}
	void FixedUpdate(){
		// Move the object at random direction at random speed
		rb.velocity = movingDirection;
	}


}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PosGenerator : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform shoulderLink;
    public float maxDistance;
    private Transform tf;
    private Rigidbody rb;
    private Vector3 initialPos;
    private Vector3 initialRot;
    private const float SPEED = 0.002f;
    void Start()
    {
        tf = GetComponent<Transform>();
        rb = GetComponent<Rigidbody>();
        initialPos = tf.position;
        initialRot = tf.eulerAngles;
    }

    void FixedUpdate()
    {
        Vector3 changedVel = Random.insideUnitSphere;
        changedVel = changedVel.normalized;
        changedVel *= SPEED/2;
        rb.velocity = rb.velocity + changedVel;
    }

    public void ResetPos()
    {
        MovingRandPos();
    }


    private void MovingRandPos()
    {
        float x_change = Random.Range(-0.02f, 0.02f);
        float y_change = Random.Range(-0.02f, 0.02f);
        float z_change = Random.Range(-0.002f, 0.002f);
        tf.position = initialPos + new Vector3(x_change, y_change, z_change);
        tf.eulerAngles = initialRot;
        Vector3 randomVel = Random.insideUnitSphere;
        randomVel = randomVel.normalized;
        randomVel *= SPEED;
        randomVel.z = -Mathf.Abs(randomVel.z); // prevent out of reach movement
        rb.velocity = randomVel;
    }
}

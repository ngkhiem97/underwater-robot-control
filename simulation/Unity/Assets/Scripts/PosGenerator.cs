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
    private int hardness = 0;
    void Start()
    {
        tf = GetComponent<Transform>();
        rb = GetComponent<Rigidbody>();
        initialPos = tf.position;
        initialRot = tf.eulerAngles;
    }

    void FixedUpdate()
    {
        if (hardness == 0)
        {
            Vector3 changedVel = Random.insideUnitSphere;
            changedVel = changedVel.normalized;
            changedVel *= 0.0025f;
            changedVel.z = 0; // prevent out of reach movement
            rb.velocity = rb.velocity + changedVel;
        }
    }

    public void ResetPos()
    {
        if (hardness == 0)
        {
            MovingRandPos();
        }
        else if (hardness == 1)
        {
            MovingRandPos();
        }
        else if (hardness == 2)
        {
            MovingRandPos();
        }
    }

    public void IncreaseHardness()
    {
        Debug.Log("Increasing hardness");
        hardness = Mathf.Min(hardness + 1, 2);
    }

    private void FixedPos() 
    {
        tf.position = initialPos;
        tf.eulerAngles = initialRot;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    private void XRandPos()
    {
        float x_change = Random.Range(-0.08f, 0.08f);
        tf.position = initialPos + new Vector3(x_change, 0, 0);
        tf.eulerAngles = initialRot;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    private void XYRandPos()
    {
        float x_change = Random.Range(-0.08f, 0.08f);
        float y_change = Random.Range(-0.08f, 0.02f);
        tf.position = initialPos + new Vector3(x_change, y_change, 0);
        tf.eulerAngles = initialRot;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    private void XYZRandPos()
    {
        float x_change = Random.Range(-0.08f, 0.08f);
        float y_change = Random.Range(-0.08f, 0.02f);
        float z_change = Random.Range(-0.01f, 0.01f);
        tf.position = initialPos + new Vector3(x_change, y_change, z_change);
        tf.eulerAngles = initialRot;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
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
        randomVel *= 0.005f;
        randomVel.z = 0; // prevent out of reach movement
        rb.velocity = randomVel;
    }
}

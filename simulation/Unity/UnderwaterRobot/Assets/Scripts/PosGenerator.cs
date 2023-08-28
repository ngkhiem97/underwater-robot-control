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
        if (hardness == 5)
        {
            Vector3 changedVel = Random.insideUnitSphere;
            changedVel = changedVel.normalized;
            changedVel *= 0.005f;
            rb.velocity = rb.velocity + changedVel;
        }
    }

    public void ResetPos()
    {
        if (hardness == 0)
        {
            Debug.Log("Resetting to fixed pos");
            FixedPos();
        }
        else if (hardness == 1)
        {
            Debug.Log("Resetting to x rand pos");
            XRandPos();
        }
        else if (hardness == 2)
        {
            Debug.Log("Resetting to xy rand pos");
            XYRandPos();
        }
        else if (hardness == 3)
        {
            Debug.Log("Resetting to xyz rand pos");
            XYZRandPos();
        } 
        else if (hardness == 4)
        {
            Debug.Log("Resetting to moving rand pos");
            MovingRandPos();
        }
        else if (hardness == 5)
        {
            Debug.Log("Resetting to moving rand pos");
            MovingRandPos();
        }
    }

    public void IncreaseHardness()
    {
        Debug.Log("Increasing hardness");
        hardness = Mathf.Min(hardness + 1, 5);
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
        float x_change = Random.Range(-0.3f, 0.3f);
        tf.position = initialPos + new Vector3(x_change, 0, 0);
        tf.eulerAngles = initialRot;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    private void XYRandPos()
    {
        float x_change = Random.Range(-0.3f, 0.3f);
        float y_change = Random.Range(-0.3f, 0.1f);
        tf.position = initialPos + new Vector3(x_change, y_change, 0);
        tf.eulerAngles = initialRot;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    private void XYZRandPos()
    {
        float x_change = Random.Range(-0.3f, 0.3f);
        float y_change = Random.Range(-0.3f, 0.1f);
        float z_change = Random.Range(-0.05f, 0.05f);
        tf.position = initialPos + new Vector3(x_change, y_change, z_change);
        tf.eulerAngles = initialRot;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    private void MovingRandPos()
    {
        float x_change = Random.Range(-0.3f, 0.3f);
        float y_change = Random.Range(-0.3f, 0.1f);
        float z_change = Random.Range(-0.05f, 0.05f);
        tf.position = initialPos + new Vector3(x_change, y_change, z_change);
        tf.eulerAngles = initialRot;
        Vector3 randomVel = Random.insideUnitSphere;
        randomVel = randomVel.normalized;
        randomVel *= 0.05f;
        rb.velocity = randomVel;
        rb.angularVelocity = Vector3.zero;
    }

    // public void GenerateRandomPos()
    // {
    //     // generate a random position around the shoulder link of max distance 1
    //     Vector3 randomPos = Random.insideUnitSphere;
    //     randomPos = randomPos.normalized;
    //     randomPos *= maxDistance;
    //     randomPos.y = Mathf.Abs(randomPos.y);
    //     randomPos.z = Mathf.Abs(randomPos.z);
    //     Vector3 combinedPos = shoulderLink.position + randomPos;
    //     if ((combinedPos.x >= -0.11 && combinedPos.x <= 0.11 && combinedPos.y >= 0.98 && combinedPos.y <= 1.11 && combinedPos.z <= 0.36) || 
    //         (combinedPos.x >= -0.11 && combinedPos.x <= 0.11 && randomPos.y >= 0 && combinedPos.y <= 1.11 && combinedPos.z <= 0.11))
    //     {
    //         this.GenerateRandomPos();
    //         return;
    //     }
    //     tf.position = combinedPos;

    //     // rotate the end effector to face the shoulder link
    //     Vector3 direction = tf.position - shoulderLink.position;
    //     Quaternion toRotation = Quaternion.FromToRotation(tf.up, direction);
    //     tf.rotation = toRotation * tf.rotation;

    //     // rotate around x -90 degrees
    //     tf.Rotate(new Vector3(-90, 0, 0), Space.Self);

    //     // set velocity to 0
    //     rb.velocity = Vector3.zero;

    //     // Set angular velocity to 0
    //     rb.angularVelocity = Vector3.zero;
    // }

    // public void OnCollisionEnter(Collision collision)
    // {
    //     Debug.Log("Collision");
    //     Debug.Log(collision.gameObject.name);
    //     Debug.Log(tf.position);
    // }

    // Update is called once per frame
    // void Update()
    // {
    //     if (count < 2)
    //     {
    //         count++;
    //     }
    //     else
    //     {
    //         count = 0;
    //         // Debug.Log("Resetting position");
    //         GenerateRandomPos();
    //     }
        
    // }
}

using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ContactReward : MonoBehaviour
{
    public GameObject agent;
    private string[] contactsStored = new string[0];
    // Start is called before the first frame update

    public void OnTriggerEnter(Collider other) 
    {
        if (other.gameObject.CompareTag("contact") && !contactsStored.Contains(other.gameObject.name))
        {
            Debug.Log("Contact made with " + other.gameObject.name);
            contactsStored.Append(other.gameObject.name);
            if (contactsStored.Length == 2)
            {
                contactsStored = new string[0];
            }
            agent.GetComponent<ArmAgent>().ContactEntered(other);
        }
    }
}
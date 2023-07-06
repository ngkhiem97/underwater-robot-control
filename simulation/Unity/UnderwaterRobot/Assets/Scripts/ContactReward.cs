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
        Debug.Log("Other name: " + other.gameObject.name);
        if (other.gameObject.CompareTag("contact") && !contactsStored.Contains(other.gameObject.name))
        {
            // add the contact to the list of contacts
            contactsStored.Append(other.gameObject.name);
            if (contactsStored.Length == 4)
            {
                contactsStored = new string[0];
            }
            agent.GetComponent<ArmAgent>().ContactEntered(other);
        }
    }
}

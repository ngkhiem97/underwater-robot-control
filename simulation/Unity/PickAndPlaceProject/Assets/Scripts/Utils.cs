using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utils
{
    public static string GetPath(string path)
    {
        return Application.dataPath + "/" + path;
    }

    // convert rotation from degrees to radians
    public static Vector3 ConvertRotation(Vector3 rotation)
    {
        return new Vector3(rotation.x * Mathf.Deg2Rad, rotation.y * Mathf.Deg2Rad, rotation.z * Mathf.Deg2Rad);
    }
}

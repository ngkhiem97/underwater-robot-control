using System;
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

    public static float ConvertRange(float value, float oldMin, float oldMax, float newMin, float newMax)
    {
        return (value - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin;
    }

    public static float CalculateMagnitude(Vector3 vector)
    {
        return Mathf.Sqrt(Mathf.Pow(vector.x, 2) + Mathf.Pow(vector.y, 2) + Mathf.Pow(vector.z, 2));
    }

    public static Vector3 ScaleToLength(Vector3 vector, float length)
    {
        float magnitude = CalculateMagnitude(vector);
        return new Vector3(vector.x * length / magnitude, vector.y * length / magnitude, vector.z * length / magnitude);
    }

    public static Vector3 ShiftVector(Vector3 vector, float x, float y, float z)
    {
        return new Vector3(vector.x + x, vector.y + y, vector.z + z);
    }

    internal static Vector3 ScaleToScale(Vector3 gripperPosition, float oldScale, float newScale)
    {
        return new Vector3(gripperPosition.x * newScale / oldScale, gripperPosition.y * newScale / oldScale, gripperPosition.z * newScale / oldScale);
    }

    internal static float GetYRotationFromXZ(float v1, float v2)
    {
        return Mathf.Atan2(v1, v2);
    }
}

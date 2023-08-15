using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;
using System.Text;
using System;

public class FloatLogSideChannel : SideChannel
{
    private GameObject target;
    public FloatLogSideChannel()
    {
        ChannelId = new Guid("621f0a70-4f87-11ea-a6bf-784f4387d1f7");
        target = GameObject.Find("Target");
    }

    protected override void OnMessageReceived(IncomingMessage msg)
    {
        float receivedFloat = msg.ReadFloat32();
        Debug.Log("From Python : " + receivedFloat);
        target.GetComponent<PosGenerator>().IncreaseHardness();
    }

    public void SendDebugStatementToPython(string logString, string stackTrace, LogType type)
    {
        if (type == LogType.Error)
        {
            var stringToSend = type.ToString() + ": " + logString + "\n" + stackTrace;
            using (var msgOut = new OutgoingMessage())
            {
                msgOut.WriteString(stringToSend);
                QueueMessageToSend(msgOut);
            }
        }
    }
}
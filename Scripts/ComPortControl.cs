using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using UnityEngine;
using UnityEngine.UI;
 
public class ComPortControl : MonoBehaviour
{
    [Header("------COM串口------")]
    public Image imgComStatus;
    public Dropdown dropdownComList;
    public Button bntOpenCom;
 
    [Header("------参数------")]
    public Button btnParamRequestList;
    public Button btnRequestParam;
    public InputField inputParamValue;
    public Button btnSetParam;
 
    SerialPort serialPort1 = null;
    MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
    bool armed = false;
    // locking to prevent multiple reads on serial port
    object readlock = new object();
    // our target sysid
    byte sysid;
    // our target compid
    byte compid;
 
    // Use this for initialization
    void Start()
    {
        //打开串口
        bntOpenCom.onClick.AddListener(new UnityEngine.Events.UnityAction(delegate ()
        {
            ClosePort();
 
            OpenPort(dropdownComList.options[dropdownComList.value].text);
        }));
 
        string[] portNames = System.IO.Ports.SerialPort.GetPortNames();
        dropdownComList.AddOptions(new List<string>(portNames));
 
        btnParamRequestList.onClick.AddListener(OnClickParamRequestList);
        btnRequestParam.onClick.AddListener(RequestParam);
        btnSetParam.onClick.AddListener(OnClickSetParam);
 
        if (portNames.Length > 0)
        {
            //OpenPort(portNames[0]);
        }
    }
 
    void OnClickParamRequestList()
    {
        if (serialPort1 == null || !serialPort1.IsOpen)
        {
            Debug.Log("请先打开串口");
            return;
        }
        MAVLink.mavlink_param_request_list_t req = new MAVLink.mavlink_param_request_list_t();
 
        req.target_system = 1;
        req.target_component = 1;
 
        byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST, req);
        Console.WriteLine("PARAM_REQUEST_LIST send");
        serialPort1.Write(packet, 0, packet.Length);
    }
 
    void OnClickSetParam()
    {
        if (serialPort1 == null || !serialPort1.IsOpen)
        {
            Debug.Log("请先打开串口");
            return;
        }
        MAVLink.mavlink_param_set_t req = new MAVLink.mavlink_param_set_t();
        byte[] ArrData3 = new byte[16];
 
        string strParamId = "RC3_MAX";
        byte[] ArrData2 = System.Text.Encoding.Default.GetBytes(strParamId);
        ArrData2.CopyTo(ArrData3, 0);
 
        req.target_system = 1;
        req.target_component = 1;
        req.param_type = (byte)(4);
        req.param_id = ArrData3;
        req.param_value = float.Parse(inputParamValue.text);
 
        byte[] reqPacket = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.PARAM_SET, req);
        Debug.Log("PARAM_SET send");
        serialPort1.Write(reqPacket, 0, reqPacket.Length);
    }
 
    void RequestParam()
    {
        if (serialPort1 == null || !serialPort1.IsOpen)
        {
            Debug.Log("请先打开串口");
            return;
        }
        MAVLink.mavlink_param_request_read_t req = new MAVLink.mavlink_param_request_read_t();
        byte[] ArrData3 = new byte[16];
 
        string strParamId = "RC3_MAX";
        byte[] ArrData2 = System.Text.Encoding.Default.GetBytes(strParamId);
        ArrData2.CopyTo(ArrData3, 0);
 
        req.target_system = 1;
        req.target_component = 1;
        req.param_index = -1;
        req.param_id = ArrData3;
 
        byte[] reqPacket = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ, req);
        Debug.Log("PARAM_Request send");
        serialPort1.Write(reqPacket, 0, reqPacket.Length);
    }
 
 
 
    IEnumerator WriteCOMData()
    {
        if (serialPort1 != null)
        {
            for (int i = 0; i < 15; i++)
            {
                Byte[] TxData = { 0xF3, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x3F };
                serialPort1.Write(TxData, 0, 8);
 
                Debug.Log(i + "次发串口指令成功");
                yield return new WaitForSeconds(0.1f);
            }
        }
        else
        {
            Debug.Log("发串口指令失败");
        }
    }
 
    #region 创建串口，并打开串口
    public void OpenPort(string portName)
    {
        string openPortName = portName;
 
        //创建串口
        int comNum = int.Parse(portName.Substring(3));
        if (comNum > 9)
        {
            openPortName = "\\\\.\\" + portName;
        }
 
 
        try
        {
            serialPort1 = new SerialPort(openPortName, 9600, Parity.None, 8, StopBits.One);
            serialPort1.ReadTimeout = 50;
            serialPort1.Open();
 
            if (serialPort1 != null && serialPort1.IsOpen)
            {
                Debug.Log(portName + "串口已打开");
                PlayerPrefs.SetString("IceBreakerComPort", portName);
                imgComStatus.sprite = Resources.Load<Sprite>("Textures/greencircle");
 
                StartCoroutine(DataReceiveFunction());
            }
            else
            {
                imgComStatus.sprite = Resources.Load<Sprite>("Textures/redcircle");
                Debug.Log(portName + "串口未打开");
            }
        }
        catch (Exception ex)
        {
            Debug.Log(ex.Message);
        }
    }
    #endregion
 
    IEnumerator DataReceiveFunction()
    {
        while (serialPort1.IsOpen)
        {
            try
            {
                MAVLink.MAVLinkMessage packet;
                lock (readlock)
                {
                    // read any valid packet from the port
                    packet = mavlink.ReadPacket(serialPort1.BaseStream);
 
                    // check its valid
                    if (packet == null || packet.data == null)
                        continue;
                }
 
                // check to see if its a hb packet from the comport
                if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                {
                    var hb = (MAVLink.mavlink_heartbeat_t)packet.data;
 
                    // save the sysid and compid of the seen MAV
                    sysid = packet.sysid;
                    compid = packet.compid;
 
                    // request streams at 2 hz
                    var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                        new MAVLink.mavlink_request_data_stream_t()
                        {
                            req_message_rate = 2,
                            req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                            start_stop = 1,
                            target_component = compid,
                            target_system = sysid
                        });
 
                    serialPort1.Write(buffer, 0, buffer.Length);
 
                    buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);
 
                    serialPort1.Write(buffer, 0, buffer.Length);
                }
 
                // from here we should check the the message is addressed to us
                if (sysid != packet.sysid || compid != packet.compid)
                    continue;
 
                //Debug.Log(packet.msgtypename);
                if (packet.msgtypename == "PARAM_VALUE")
                {
                    var ack = (MAVLink.mavlink_param_value_t)packet.data;
                    string id = System.Text.Encoding.Default.GetString(ack.param_id).Trim('\0');
                    Debug.Log(string.Format("{0}, {1}, {2}, {3}, {4}", ack.param_count, ack.param_index, ack.param_type, id, ack.param_value));
 
                    //if (id == "RC3_MAX")
                    //{
                    //    Debug.Log(string.Format("-------{0}, {1}, {2}, {3}, {4}", ack.param_count, ack.param_index, ack.param_type, id, ack.param_value));
                    //}
                }
            }
            catch (Exception ex)
            {
                //Debug.LogError(ex.Message);
            }
 
            yield return null;
        }
    }
 
    #region 程序退出时关闭串口
    public void ClosePort()
    {
        if (serialPort1 == null)
            return;
 
        try
        {
            serialPort1.Close();
            //dataReceiveThread.Abort();
        }
        catch (Exception ex)
        {
            Debug.Log(ex.Message);
        }
    }
    #endregion
 
    /// 查找最后一个字符出现的索引
    /// </summary>
    /// <param name="buffer"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    private int ByteArrayReverseIndex(byte[] buffer, byte b)
    {
        int index = -1;
        for (int i = 0; i < buffer.Length; i++)
        {
            if (buffer[i] == b)
            {
                index = i;
            }
        }
 
        return index;
    }
 
    void OnDestroy()
    {
        ClosePort();
    }
 
    /// <summary>
    /// 获取下拉列表某项的索上
    /// </summary>
    /// <param name="dd">下拉列表</param>
    /// <param name="str">某项文字</param>
    /// <returns>索引</returns>
    public static int GetDropdownIndex(Dropdown dd, string str)
    {
        int i = 0;
        int index = 0;
        foreach (var op in dd.options)
        {
            if (op.text.Equals(str))
            {
                index = i;
                break;
            }
            i++;
        }
        return index;
    }
 
    private static string byteToHexStr(byte[] bytes, int length)
    {
        string returnStr = "";
        if (bytes != null)
        {
            for (int i = 0; i < length; i++)
            {
                returnStr += bytes[i].ToString("X2");
            }
        }
        return returnStr;
    }
}

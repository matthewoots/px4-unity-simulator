/*

Created by Matthew Woo 2022
Contact: matthewoots@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

*/

using System;
using System.Collections; 
using System.Collections.Generic; 
using System.Net; 
using System.Net.Sockets; 
using System.Text; 
using System.Threading; 
using UnityEngine;  

public class TCPTestServer : MonoBehaviour {  	
	#region private members 	
	/// <summary> 	
	/// TCPListener to listen for incomming TCP connection 	
	/// requests. 	
	/// </summary> 	
	private TcpListener tcpListener; 
	/// <summary> 
	/// Background thread for TcpServer workload. 	
	/// </summary> 	
	private Thread tcpListenerThread;  	
	/// <summary> 	
	/// Create handle to connected tcp client. 	
	/// </summary> 	
	private TcpClient connectedTcpClient; 	
	#endregion 	

	private bool isOpen;

    //the name of the connection, not required but better for overview if you have more than 1 connections running
	public string conName = "localhost";
	
	//ip/address of the server, 127.0.0.1 is for your own computer
	public string conHost = "127.0.0.1";
	
	//port for the server, make sure to unblock this in your router firewall if you want to allow external connections
	public int conPort = 4560;

	MAVLink.mavlink_heartbeat_t Hb;
	MAVLink.mavlink_hil_actuator_controls_t controls;
	MAVLink.mavlink_hil_gps_t gps;
	MAVLink.mavlink_hil_sensor_t sensors;
	MAVLink.mavlink_hil_state_quaternion_t state;
		
	// Use this for initialization
	void Start () { 		
		isOpen = true;
		// Start TcpServer background thread 		
		tcpListenerThread = new Thread (new ThreadStart(ListenForIncommingRequests)); 		
		tcpListenerThread.IsBackground = true; 		
		tcpListenerThread.Start(); 	
	}  	
	
	// Update is called once per frame
	void Update () { 
		// Don't send until sensor suite is stabilized, which means that it must initialized to send home state to PX4
		if (!gameObject.GetComponent<SensorSuite>().init)
			return;

		SendMessage();         
	}  	
	
	/// <summary> 	
	/// Runs in background TcpServerThread; Handles incomming TcpClient requests 	
	/// </summary> 	
	private void ListenForIncommingRequests () { 	
		try { 			
			// Create listener on localhost port 8052. 			
			tcpListener = new TcpListener(IPAddress.Parse(conHost), conPort); 			
			tcpListener.Start();    
			// Server starting          
			Debug.Log("Server is listening");              
			Byte[] bytes = new Byte[1024];  			
			while (isOpen) { 				
                MAVLink.MAVLinkMessage packet;
				using (connectedTcpClient = tcpListener.AcceptTcpClient()) { 					
					// Get a stream object for reading 					
					using (NetworkStream stream = connectedTcpClient.GetStream()) { 
                        MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
                        packet = mavlink.ReadPacket(stream);	
						int length = packet.Length;
						
                        // Read incomming stream into byte arrary. 						
                        while ((length = stream.Read(bytes, 0, bytes.Length)) != 0) { 							
                        	// check its valid
							if (packet == null || packet.data == null)
								continue;	
							uint msgid = packet.msgid;
							Debug.Log("client msgid " + msgid);					
                        } 					
					} 				
				} 			
			} 		
		} 		
		catch (SocketException socketException) { 			
			Debug.Log("SocketException " + socketException.ToString()); 		
		}     
	}  	
	/// <summary> 	
	/// Send message to client using socket connection. 	
	/// </summary> 	
	private void SendMessage() { 		
		if (connectedTcpClient == null) {             
			return;         
		}  		
		
		try { 			
			MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
			sensors = gameObject.GetComponent<SensorSuite>()._sensor_data;
			state = gameObject.GetComponent<SensorSuite>()._state_data;
			gps = gameObject.GetComponent<SensorSuite>()._gps_data;
			byte[] buffer_sensors = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HIL_SENSOR, sensors);
			byte[] buffer_state = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HIL_STATE, state);
			byte[] buffer_gps = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HIL_GPS, gps);
			NetworkStream stream = connectedTcpClient.GetStream();

			stream.Write(buffer_gps, 0, buffer_gps.Length); 
			stream.Write(buffer_sensors, 0, buffer_sensors.Length);   
			stream.Write(buffer_state, 0, buffer_state.Length);   
			         
			Debug.Log("Server sent his message - should be received by client");
			// // Get a stream object for writing. 			
			// NetworkStream stream = connectedTcpClient.GetStream(); 			
			// if (stream.CanWrite) {                 
			// 	string serverMessage = "This is a message from your server."; 			
			// 	// Convert string message to byte array.                 
			// 	byte[] serverMessageAsByteArray = Encoding.ASCII.GetBytes(serverMessage); 				
			// 	// Write byte array to socketConnection stream.               
			// 	stream.Write(serverMessageAsByteArray, 0, serverMessageAsByteArray.Length);               
			// 	Debug.Log("Server sent his message - should be received by client");           
			// }       
		} 		
		catch (SocketException socketException) {             
			Debug.Log("Socket exception: " + socketException);         
		} 	
	} 

	void OnApplicationQuit()
	{
		isOpen = false;
		tcpListener.Stop();
		// wait for listening thread to terminate (max. 500ms)
		tcpListenerThread.Join(500);
	}
}
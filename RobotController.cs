using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider FLC;
    [SerializeField] private WheelCollider FRC;
    [SerializeField] private WheelCollider RLC;
    [SerializeField] private WheelCollider RRC;

    [SerializeField] private Transform FLT;
    [SerializeField] private Transform FRT;
    [SerializeField] private Transform RLT;
    [SerializeField] private Transform RRT;

    [SerializeField] private Transform FRS;
    [SerializeField] private Transform L1S;
    [SerializeField] private Transform L2S;
    [SerializeField] private Transform L3S;
    [SerializeField] private Transform R1S;
    [SerializeField] private Transform R2S;
    [SerializeField] private Transform R3S;
    [SerializeField] private Transform ORS;


    [Header("Movement Parameters")]
    public float motorTorque = 2000f;
    public float maxSpeed = 50f;
    public float turnSpeed = 30f;

    [Header("Sensor Parameters")]
    public float sensorRange = 10f;
    public float obstacleDetectionDistance = 3f;
    public string roadMaterial = "MT_Road_01";

    [Header("Performance Tuning")]
    public float steeringSmoothing = 5f;
    public float accelerationSmoothing = 2f;

    private float currentSteerAngle = 0f;
    private float currentMotorTorque = 0f;

    private float episodeTime = 0f; // Track the elapsed time since the episode started
    private const float maxEpisodeTime = 2500f; // Max episode time in seconds (120 seconds)


    [Header("Movement Parameters")]
    public float maxMotorTorque = 400f;
    public float maxSteeringAngle = 60f;
    private GeneticAlgorithm ga;
    private int currentStep = 0;
    private int sequenceLength = 500;
    private float totalReward = 0f;  // To accumulate fitness score

    private void Start()
    {
        ga = FindObjectOfType<GeneticAlgorithm>();
    }


    // private void FixedUpdate()
    // {
    //     if (currentStep < sequenceLength)
    //     {
    //         // Debug.Log($"Current Step: {currentStep}, Sequence Length: {sequenceLength}, Gene Length: {ga.population[0].Count}");

    //         // Get motorTorque and steeringAngle from GA
    //         float motorTorque = ga.population[0][currentStep].x * maxMotorTorque;
    //         float steeringAngle = ga.population[0][currentStep].y * maxSteeringAngle;

    //         Debug.Log($"Motor Torque: {motorTorque}, Steering Angle: {steeringAngle}, currentStep: {currentStep}");

    //         ApplyMotorTorque(motorTorque);
    //         ApplySteering(steeringAngle);
    //         UpdateWheelTransforms();

    //         // Calculate and accumulate fitness using the HandleTrackRewards function
    //         float reward = HandleTrackRewards(motorTorque, steeringAngle);
    //         totalReward += reward;

    //         currentStep++;
    //     }
    //     else
    //     {
    //         //first position
    //         transform.localPosition = new Vector3(195.6539f, 0.6679955f, 192.1293f);
    //         transform.rotation = Quaternion.Euler(0f, 180f, 0f);
    //         currentStep = 0;  // Reset step counter
    //         ga.UpdateFitness(totalReward);  // Pass accumulated reward to GA as fitness
    //         totalReward = 0f;  // Reset accumulated reward
    //     }
    // }

    public void OnEpisodeBegin()
    {
        // Reset the episode timer at the beginning of each episode
        episodeTime = 0f;

        Debug.Log("Episode begining");

        // Stop the robot's velocity and angular velocity to prevent movement at the start
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Stop all wheels from rotating by setting motorTorque to zero
        FLC.motorTorque = 0f;
        FRC.motorTorque = 0f;
        RLC.motorTorque = 0f;
        RRC.motorTorque = 0f;

        // Reset wheel steer angles to prevent unintended turning
        FLC.steerAngle = 0f;
        FRC.steerAngle = 0f;

        // transform.localPosition = new Vector3(194.7755f, 0.6679955f, -153.1348f);
        // Reset robot position and rotation as you have it in your current code

        //first position
        transform.localPosition = new Vector3(195.6539f, 0.6679955f, 192.1293f);
        transform.rotation = Quaternion.Euler(0f, 180f, 0f);

        // third position
        // transform.localPosition = new Vector3(63.20256f, 14.68702f, -123.3367f);
        // transform.rotation = Quaternion.Euler(10.608f, 359.733f, -1.223f);

        //fifth position
        // transform.localPosition = new Vector3(66.54259f, 16.45044f, -35.98968f);
        // transform.rotation = Quaternion.Euler(-0.491f, 10.278f, -0.985f);

        // fourth position
        // transform.localPosition = new Vector3(23.57335f, 16.44876f, -23.40142f);
        // transform.rotation = Quaternion.Euler(-0.08f, 187.596f, 0.571f);

        // Set sensor orientations as defined
        SetSensorOrientations();

        // Debug.Log("Episode has started");
    }

    public void Heuristic(in ActionBuffers actionOut)
    {
        ActionSegment<float> continuousActions = actionOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Vertical") * .5f;
        continuousActions[1] = Input.GetAxisRaw("Horizontal") * 15;
    }

    public float HandleTrackRewards(float motorTorque, float steeringAngle)
    {
        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);
        float reward = 0f;  // Reward accumulator

        if (speed > 0f)
        {
            reward += speed > 1.5f ? (speed < 5.5f ? 0.1f : -0.1f) : -0.1f;
        }
        else
        {
            reward -= 1f;
        }

        var sensorReadings = GetSensorData();
        foreach (var sensorReading in sensorReadings)
        {
            float distance = sensorReading.Value.Item1;
            string hitObject = sensorReading.Value.Item2;

            if (hitObject.Contains("Cube") && distance < 1f)
            {
                reward -= 0.2f;
            }
            else if (sensorReading.Key == "Left3" || sensorReading.Key == "Right3")
            {
                reward += 0.025f * distance;
            }
        }

        if (IsOutOfTrack())
        {
            reward -= 10f;
        }

        return reward;  // Return accumulated reward
    }


    // Add these methods to RobotController.cs

    // Method to reset the car without triggering ML-Agents logic
    public void ManualReset()
    {
        // Reset position, rotation, and physics
        transform.localPosition = new Vector3(195.6539f, 0.6679955f, 192.1293f);
        transform.rotation = Quaternion.Euler(0f, 180f, 0f);
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        SetSensorOrientations();
    }

    // Method to directly apply torque/steering (bypass ML-Agents actions)
    public void ManualApplyControl(float torque, float steering)
    {
        ApplySteering(steering);
        ApplyMotorTorque(torque);
        UpdateWheelTransforms();
    }

    // Method to get the distance covered (for GA fitness)
    public float GetDistanceCovered()
    {
        return transform.position.z; // Adjust based on your track layout
    }

    private bool CheckForTurningPoint(Dictionary<string, (float, string)> sensorReadings)
    {
        return sensorReadings["Front"].Item2.StartsWith("MT_Turn") ||
               sensorReadings["Left1"].Item2.StartsWith("MT_Turn") ||
               sensorReadings["Left2"].Item2.StartsWith("MT_Turn") ||
               sensorReadings["Right1"].Item2.StartsWith("MT_Turn") ||
               sensorReadings["Right2"].Item2.StartsWith("MT_Turn");
    }

    private bool CheckForCheckpointInFront(Dictionary<string, (float, string)> sensorReadings)
    {
        return sensorReadings["Front"].Item2.StartsWith("CP") ||
               sensorReadings["Right2"].Item2.StartsWith("CP") ||
               sensorReadings["Left1"].Item2.StartsWith("CP");
    }

    private void HandleEdgeDetection(Dictionary<string, (float, string)> sensorReadings, float steeringAngle, float motorTorque)
    {
        bool leftOverEdge = (sensorReadings["Left1"].Item2.StartsWith("None") &&
                                sensorReadings["Left2"].Item2.StartsWith("None") &&
                                sensorReadings["Left3"].Item2.StartsWith("None") &&
                                sensorReadings["Right1"].Item2.StartsWith("MT_Road") &&
                                sensorReadings["Right2"].Item2.StartsWith("MT_Road") &&
                                sensorReadings["Right3"].Item2.StartsWith("MT_Road") &&
                                (sensorReadings["Front"].Item2.StartsWith("MT_Road") || sensorReadings["Front"].Item2.StartsWith("ED")));

        bool rightOverEdge = (sensorReadings["Right1"].Item2.StartsWith("None") &&
                                sensorReadings["Right2"].Item2.StartsWith("None") &&
                                sensorReadings["Right3"].Item2.StartsWith("None") &&
                                sensorReadings["Left1"].Item2.StartsWith("MT_Road") &&
                                sensorReadings["Left2"].Item2.StartsWith("MT_Road") &&
                                sensorReadings["Left3"].Item2.StartsWith("MT_Road") &&
                                sensorReadings["Front"].Item2.StartsWith("MT_Road"));


        bool leftEdgeDetected = (sensorReadings["Left3"].Item2.StartsWith("ED") && sensorReadings["Left3"].Item1 < 2.5f) || leftOverEdge;


        bool rightEdgeDetected = (sensorReadings["Right3"].Item2.StartsWith("ED") && sensorReadings["Right3"].Item1 < 2.5f) || rightOverEdge;


        if (leftEdgeDetected || rightEdgeDetected)
        {
            // AddReward(-2f);

            //Remove after training
            // if (leftOverEdge || rightOverEdge)
            // {
            //     EndEpisode();
            // }

        }

        float deviation = sensorReadings["Left3"].Item1 - sensorReadings["Right3"].Item1;
        if (Mathf.Abs(deviation) < 0.5f && sensorReadings["Left3"].Item2.StartsWith("ED") && sensorReadings["Right3"].Item2.StartsWith("ED"))
        {
            // AddReward(0.0001f);
        }


    }

    private void HandleCheckpointRewards(Dictionary<string, (float, string)> sensorReadings)
    {
        for (int i = 1; i <= 22; i++)
        {
            string checkpointName = "CP" + i;
            if (CheckForCheckpointPassed(checkpointName))
            {
                // AddReward(0.2f * i);
                break;
            }
        }
    }

    public bool HandleFinalCheckpoint()
    {
        if (HasPassedFinalCheckpoint() && IsStopped())
        {
            // AddReward(1f);
            // Debug.Log("EndEpisode: Successfully completed");
            // EndEpisode();
            return true;
        }
        return false;
    }


    public bool IsOutOfTrack()
    {
        // Check if the robot has moved outside the allowed track area
        // You can define track boundaries based on the road material or specific coordinates
        // Here, assuming the robot has gone out of bounds of the road material

        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 2f); // Adjust radius as necessary
        foreach (var collider in hitColliders)
        {
            // Debug.Log($"collider object: {collider.gameObject.name}");


            if (collider.gameObject.name.StartsWith("ED"))
                return true;

            if ((collider.gameObject.name.StartsWith("MT_Road") || collider.gameObject.name.StartsWith("MT_Turn")))
            {
                // Debug.Log($"collider object: {collider.gameObject.name}");
                return false; // The robot is on the road, not out of track
            }
            ;
        }

        // If no road material is found, the robot is out of the track
        return true;
    }

    private bool CheckForCheckpointPassed(string checkpointName)
    {
        // Check if the robot has passed a checkpoint
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 1f); // Adjust radius as needed
        foreach (var collider in hitColliders)
        {
            if (collider.gameObject.name.StartsWith(checkpointName))
            {
                return true; // The robot has passed this checkpoint
            }
        }
        return false;
    }

    public bool HasPassedFinalCheckpoint()
    {
        // Check if the robot has passed CP22
        return CheckForCheckpointPassed("CP22");
    }

    private bool IsStopped()
    {
        // Check if the robot has stopped moving (velocity is low)
        return GetComponent<Rigidbody>().linearVelocity.magnitude < 0.1f; // Adjust as necessary for "stopped" state
    }

    public void CollectObservations(VectorSensor sensor)
    {
        var sensorReadings = GetSensorData();
        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);
        int frontItemId = GetHitItemId(sensorReadings["Front"].Item2);
        sensor.AddObservation(frontItemId);
        sensor.AddObservation(sensorReadings["Front"].Item1);
        int left1ItemId = GetHitItemId(sensorReadings["Left1"].Item2);
        sensor.AddObservation(left1ItemId);
        sensor.AddObservation(sensorReadings["Left1"].Item1);
        int Left2ItemId = GetHitItemId(sensorReadings["Left2"].Item2);
        sensor.AddObservation(Left2ItemId);
        sensor.AddObservation(sensorReadings["Left2"].Item1);
        int Left3ItemId = GetHitItemId(sensorReadings["Left3"].Item2);
        sensor.AddObservation(Left3ItemId);
        sensor.AddObservation(sensorReadings["Left3"].Item1);
        int Right1ItemId = GetHitItemId(sensorReadings["Right1"].Item2);
        sensor.AddObservation(Right1ItemId);
        sensor.AddObservation(sensorReadings["Right1"].Item1);
        int Right2ItemId = GetHitItemId(sensorReadings["Right2"].Item2);
        sensor.AddObservation(Right2ItemId);
        sensor.AddObservation(sensorReadings["Right2"].Item1);
        int Right3ItemId = GetHitItemId(sensorReadings["Right3"].Item2);
        sensor.AddObservation(Right3ItemId);
        sensor.AddObservation(sensorReadings["Right3"].Item1);
        sensor.AddObservation(sensorReadings["ORS"].Item1);
        sensor.AddObservation(sensorReadings["ORSZ"].Item1);
        sensor.AddObservation(speed);

    }

    private int GetHitItemId(string hitName)
    {
        if (hitName.StartsWith("MT_Road"))
            return 1;
        else if (hitName.StartsWith("MT_Turn"))
            return 2;
        else if (hitName.StartsWith("CP"))
            return 3;
        else if (hitName.StartsWith("Plane"))
            return 4;
        else if (hitName.StartsWith("ED"))
            return 5;
        else if (hitName.StartsWith("Cube"))
            return 6;
        else
            return 0;
    }

    private void SetSensorOrientations()
    {
        FRS.localRotation = Quaternion.Euler(8, 0, 0);
        L1S.localRotation = Quaternion.Euler(8, -15, 0);
        L2S.localRotation = Quaternion.Euler(8, -35, 0);
        L3S.localRotation = Quaternion.Euler(10, -90, 0);
        R1S.localRotation = Quaternion.Euler(8, 15, 0);
        R2S.localRotation = Quaternion.Euler(8, 35, 0);
        R3S.localRotation = Quaternion.Euler(10, 90, 0);
    }

    private Dictionary<string, (float, string)> GetSensorData()
    {
        return new Dictionary<string, (float, string)>
        {
            { "Front", CheckSensor(FRS) },
            { "Left1", CheckSensor(L1S) },
            { "Left2", CheckSensor(L2S) },
            { "Left3", CheckSensor(L3S) },
            { "Right1", CheckSensor(R1S) },
            { "Right2", CheckSensor(R2S) },
            { "Right3", CheckSensor(R3S) },
            { "ORS", CheckOrientationSensor() },
            { "ORSZ", CheckOrientationSensorZ() }
        };
    }

    private (float, string) CheckOrientationSensorZ()
    {
        // Get the robot's forward-facing angle relative to the world
        float zaw = transform.eulerAngles.z;
        // Debug.Log($"xaw pitch: {xaw}");
        // Normalize the pitch
        float normalizedPitch = (zaw > 180) ? zaw - 360 : zaw;

        // Return the xaw value along with a descriptor
        return (normalizedPitch, "OrientationZ");
    }

    private (float, string) CheckOrientationSensor()
    {
        // Get the robot's forward-facing angle relative to the world
        float xaw = transform.eulerAngles.x;
        // Debug.Log($"xaw pitch: {xaw}");
        // Normalize the pitch
        float normalizedPitch = (xaw > 180) ? xaw - 360 : xaw;

        // Return the xaw value along with a descriptor
        return (normalizedPitch, "OrientationX");
    }



    private (float, string) CheckSensor(Transform sensor)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensor.position, sensor.forward, out hit, sensorRange))
        {
            Debug.DrawLine(sensor.position, hit.point, Color.red);
            return (hit.distance, hit.collider.gameObject.name);
        }
        Debug.DrawLine(sensor.position, sensor.position + sensor.forward * sensorRange, Color.green);
        return (sensorRange, "None");
    }

    public void ApplySteering(float targetAngle)
    {
        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetAngle, Time.deltaTime * steeringSmoothing);
        FLC.steerAngle = currentSteerAngle;
        FRC.steerAngle = currentSteerAngle;
    }

    public void ApplyMotorTorque(float targetTorque)
    {
        currentMotorTorque = Mathf.Lerp(currentMotorTorque, targetTorque, Time.deltaTime * accelerationSmoothing);
        FLC.motorTorque = currentMotorTorque;
        FRC.motorTorque = currentMotorTorque;
        RLC.motorTorque = currentMotorTorque;
        RRC.motorTorque = currentMotorTorque;
    }

    public void UpdateWheelTransforms()
    {
        UpdateWheelTransform(FLC, FLT);
        UpdateWheelTransform(FRC, FRT);
        UpdateWheelTransform(RLC, RLT);
        UpdateWheelTransform(RRC, RRT);
    }

    private void UpdateWheelTransform(WheelCollider collider, Transform wheelTransform)
    {
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
        wheelTransform.position = position;
        wheelTransform.rotation = rotation;
    }
}


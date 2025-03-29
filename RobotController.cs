using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    // Wheel Colliders and Transforms
    [SerializeField] private WheelCollider FLC, FRC, RLC, RRC;
    [SerializeField] private Transform FLT, FRT, RLT, RRT;

    // Sensor Transforms
    [SerializeField] private Transform FRS, L1S, L2S, L3S, R1S, R2S, R3S, ORS;

    // Movement Parameters
    [Header("Movement Parameters")]
    public float motorTorque = 2000f;
    public float maxSpeed = 50f;
    public float turnSpeed = 30f;
    public float maxMotorTorque = 400f;
    public float maxSteeringAngle = 60f;

    // Sensor Parameters
    [Header("Sensor Parameters")]
    public float sensorRange = 10f;
    public float obstacleDetectionDistance = 3f;
    public string roadMaterial = "MT_Road_01";

    // Performance Tuning
    [Header("Performance Tuning")]
    public float steeringSmoothing = 500000f;
    public float accelerationSmoothing = 50000f;

    // Runtime Variables
    private float currentSteeringAngle = 0f;
    private float currentMotorTorque = 0f;
    private GeneticAlgorithm ga;
    private int individualIndex;
    private float totalReward;
    private List<Vector2> currentIndividual;
    private bool isActive;
    public bool shouldRender = true; // Controls visual updates

    // Initialization
    private void Start()
    {
        ga = FindObjectOfType<GeneticAlgorithm>();
    }

    public void InitializeForGA(GeneticAlgorithm geneticAlgorithm, int index)
    {
        ga = geneticAlgorithm;
        individualIndex = index;
        totalReward = 0f;
        isActive = true;
        ManualReset();
    }

    public void SetIndividual(List<Vector2> individual)
    {
        currentIndividual = individual;
        totalReward = 0f;
        isActive = true;
        // ✅ Reset steering and torque for fresh generation
        currentMotorTorque = 0f;
        currentSteeringAngle = 0f;
    }

    // Update Fitness
    public void UpdateFitness(bool checkSpeed)
    {
        if (!isActive) return;

        float reward = HandleTrackRewards(currentMotorTorque, currentSteeringAngle);
        // Debug.Log($"Motor torque: {currentMotorTorque}, steer: {currentSteeringAngle}");
        totalReward += reward;

        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);

        if (IsOutOfTrack() || (checkSpeed && speed <= 1f))
        {
            ga.UpdateFitness(individualIndex, totalReward, true);
            isActive = false;
            GetComponent<Rigidbody>().isKinematic = true;
        }
        else if (HandleFinalCheckpoint())
        {
            ga.UpdateFitness(individualIndex, totalReward, true);
            isActive = false;
            GetComponent<Rigidbody>().isKinematic = true;
        }
    }

    // Calculate Track Rewards
    public float HandleTrackRewards(float motorTorque, float steeringAngle)
    {
        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);
        float reward = 0f;

        if (speed > 0f)
        {
            reward += speed > 2f ? (speed < 6f ? 1f : -0.3f) : -0.3f;
        }
        else
        {
            reward -= 1f;
        }

        // if((isOnTurn(1) || isOnTurn(2)) && steeringAngle > 0) {
        //     Debug.Log("Turning rewards adding");
        //     reward += 2f;
        // }

        var sensorReadings = GetSensorData();
        foreach (var sensorReading in sensorReadings)
        {
            float distance = sensorReading.Value.Item1;
            string hitObject = sensorReading.Value.Item2;

            if (sensorReading.Key == "Left1" || sensorReading.Key == "Front" || sensorReading.Key == "Right1")
            {
                if ((hitObject.Contains("MT_Turn (1)") || hitObject.Contains("MT_Turn (2)")) && steeringAngle > 20f)
                {
                    Debug.Log("Turning rewards adding");
                    reward += 5f;
                }
            }


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

        return reward;
    }

    // Manual Reset
    public void ManualReset()
    {
        // transform.localPosition = new Vector3(195.6539f, 0.6679955f, 192.1293f); //first env location
        // transform.localPosition = new Vector3(195.6539f, 0.6679955f, -68f); // second env location 
        // transform.localPosition = new Vector3(195.6539f, 0.6679955f, -147f); // second env location 
        transform.localPosition = new Vector3(195.6539f, 0.6679955f, -105f); // second env location 
        transform.rotation = Quaternion.Euler(0f, 180f, 0f);
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.isKinematic = false;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        // Sleep & wake for clean reset
        rb.Sleep();
        rb.WakeUp();
        currentMotorTorque = 0f;
        currentSteeringAngle = 0f;
        SetSensorOrientations();
    }

    // Manual Control Application
    public void ManualApplyControl(float torque, float steering)
    {
        ApplySteering(steering);
        ApplyMotorTorque(torque);
        if (shouldRender)
        {
            UpdateWheelTransforms();
        }
    }

    // Check if on Turn
    public bool isOnTurn(int turn = 0)
    {
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 2f);
        foreach (var collider in hitColliders)
        {
            string turning = turn > 0 ? " (" + turn + ")" : "";
            if (collider.gameObject.name.StartsWith("MT_Turn" + turning))
            {
                return true;
            }
        }
        return false;
    }

    // Check if Out of Track
    public bool IsOutOfTrack()
    {
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 2f);
        foreach (var collider in hitColliders)
        {
            if (collider.gameObject.name.StartsWith("ED"))
                return true;
            if (collider.gameObject.name.StartsWith("MT_Road") || collider.gameObject.name.StartsWith("MT_Turn"))
                return false;
        }
        return true;
    }

    // Handle Final Checkpoint
    public bool HandleFinalCheckpoint()
    {
        return HasPassedFinalCheckpoint() && IsStopped();
    }

    private bool HasPassedFinalCheckpoint()
    {
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 1f);
        foreach (var collider in hitColliders)
        {
            if (collider.gameObject.name.StartsWith("CP22"))
                return true;
        }
        return false;
    }

    private bool IsStopped()
    {
        return GetComponent<Rigidbody>().linearVelocity.magnitude < 0.1f;
    }

    // Sensor Data Collection
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

    private (float, string) CheckOrientationSensor()
    {
        float xaw = transform.eulerAngles.x;
        float normalizedPitch = (xaw > 180) ? xaw - 360 : xaw;
        return (normalizedPitch, "OrientationX");
    }

    private (float, string) CheckOrientationSensorZ()
    {
        float zaw = transform.eulerAngles.z;
        float normalizedPitch = (zaw > 180) ? zaw - 360 : zaw;
        return (normalizedPitch, "OrientationZ");
    }

    private (float, string) CheckSensor(Transform sensor)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensor.position, sensor.forward, out hit, sensorRange))
        {
            return (hit.distance, hit.collider.gameObject.name);
        }
        return (sensorRange, "None");
    }

    // Apply Steering
    public void ApplySteering(float targetAngle)
    {
        // Debug.Log($"currentSteeringAngle : {currentSteeringAngle}, targetAngle : {targetAngle}");
        // currentSteeringAngle = Mathf.Lerp(currentSteeringAngle, targetAngle, Time.deltaTime * 2f);
        currentSteeringAngle = targetAngle;
        // Debug.Log($"After clamp currentSteeringAngle : {currentSteeringAngle}, targetAngle : {targetAngle}");
        FLC.steerAngle = currentSteeringAngle;
        FRC.steerAngle = currentSteeringAngle;
    }

    // Apply Motor Torque
    public void ApplyMotorTorque(float targetTorque)
    {
        // Debug.Log($"currentMotorTorque : {currentMotorTorque}, targetTorque : {targetTorque}");
        // currentMotorTorque = Mathf.Lerp(currentMotorTorque, targetTorque, Time.deltaTime * accelerationSmoothing * 10f);
        currentMotorTorque = targetTorque;
        // Debug.Log($"After clamp currentMotorTorque : {currentMotorTorque}, targetTorque : {targetTorque}");
        FLC.motorTorque = currentMotorTorque;
        FRC.motorTorque = currentMotorTorque;
        RLC.motorTorque = currentMotorTorque;
        RRC.motorTorque = currentMotorTorque;
    }

    // Update Wheel Transforms
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

    // Set Sensor Orientations
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
}
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    // Wheel Colliders and Transforms
    [SerializeField] private WheelCollider FLC, FRC, RLC, RRC;
    [SerializeField] private Transform FLT, FRT, RLT, RRT;

    // Sensor Transforms
    [SerializeField] private Transform FRS, L1S, L2S, L3S, R1S, R2S, R3S, ORS, Down;

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

    private float totalTorqueReward = 0f;
    private float totalSteeringReward = 0f;


    // Initialization
    private void Start()
    {
        ga = FindObjectOfType<GeneticAlgorithm>();
    }

    public void InitializeForGA(GeneticAlgorithm geneticAlgorithm, int index)
    {
        ga = geneticAlgorithm;
        individualIndex = index;
        totalSteeringReward = 0f;
        totalTorqueReward = 0f;
        isActive = true;
        ManualReset();
    }

    public void SetIndividual(List<Vector2> individual)
    {
        currentIndividual = individual; ;
        totalSteeringReward = 0f;
        totalTorqueReward = 0f;
        isActive = true;
        // ✅ Reset steering and torque for fresh generation
        currentMotorTorque = 0f;
        currentSteeringAngle = 0f;
    }

    // Update Fitness
    public void UpdateFitness(bool checkSpeed)
    {
        if (!isActive) return;

        float torqueReward = HandleTorqueRewards(currentMotorTorque);
        float steeringReward = HandleSteeringRewards(currentSteeringAngle);
        // Debug.Log($"Motor torque: {currentMotorTorque}, steer: {currentSteeringAngle}");
        totalTorqueReward += torqueReward;
        totalSteeringReward += steeringReward;

        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);

        if (IsOutOfTrack() || (checkSpeed && speed <= 1f))
        {
            ga.UpdateFitness(individualIndex, totalTorqueReward, totalSteeringReward, true);
            isActive = false;
            GetComponent<Rigidbody>().isKinematic = true;
        }
        else if (HandleFinalCheckpoint())
        {
            ga.UpdateFitness(individualIndex, totalTorqueReward, totalSteeringReward, true);
            isActive = false;
            GetComponent<Rigidbody>().isKinematic = true;
        }
    }

    public float HandleTorqueRewards(float motorTorque)
    {
        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);
        float reward = 0f;

        if (speed > 0f)
        {
            reward += speed > 2f ? (speed < 6f ? 1f : -0.3f) : -0.1f;
        }
        else
        {
            reward -= 1f;
        }

        if (IsOutOfTrack())
        {
            reward -= 10f;
        }

        return reward;
    }

    public float HandleSteeringRewards(float steeringAngle)
    {
        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);
        float reward = 0f;

        if (GetRoad() == 1)
        {
            if (steeringAngle > 5f)
            {
                // Debug.Log("Turning rewards adding");
                reward += steeringAngle > 20f ? 5f : 2f;
                reward += steeringAngle > 30f ? 10f : 0f;
                reward += steeringAngle > 40f ? 12f : 0f;
            }else
                reward += -5f;
        }

        if (GetRoad() == 0 && steeringAngle > -10f && steeringAngle < 10f)
            reward += 1f;

        return reward;
    }

    public int GetRoad()
    {
        var sensorReadings = GetSensorData();
        string[] keysToCheck = { "Down", };

        foreach (var key in keysToCheck)
        {
            string hitObject = sensorReadings[key].Item2;
            // Debug.Log($"Hitobject: {hitObject}");

            if (hitObject.Contains("MT_Turn (1)") || hitObject.Contains("MT_Turn (2)"))
            {
                return 1;
            }
        }

        return 0;
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
        // Debug.Log($"Torque: {torque}, steering: {currentSteeringAngle}");
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
        var sensorReadings = GetSensorData();
        string hitObject = sensorReadings["Down"].Item2;

        // Debug.Log($"Down hit: {hitObject}");

        if (hitObject.StartsWith("MT_Road") || hitObject.StartsWith("MT_Turn"))
            return false; // ✅ Still on track

        if (hitObject.StartsWith("ED"))
            return true; // ❌ Explicitly off-track

        return true; // ❌ Anything else = unknown = off-track
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
            { "Front", CheckSensor(FRS, true) },
            { "Left1", CheckSensor(L1S, true) },
            { "Left2", CheckSensor(L2S) },
            { "Left3", CheckSensor(L3S) },
            { "Right1", CheckSensor(R1S, true) },
            { "Right2", CheckSensor(R2S) },
            { "Right3", CheckSensor(R3S) },
            { "Down", CheckSensor(Down) },
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

    private (float, string) CheckSensor(Transform sensor, bool draw = false)
    {
        RaycastHit hit;

        // Exclude 'Robot' layer
        int layerMask = ~LayerMask.GetMask("Robot");

        // ✅ Only draw if this is the Down sensor
        if (draw)
        {
            Debug.DrawRay(sensor.position, sensor.forward * sensorRange, Color.yellow);
        }

        if (Physics.Raycast(sensor.position, sensor.forward, out hit, sensorRange, layerMask))
        {
            return (hit.distance, hit.collider.gameObject.name);
        }
        return (sensorRange, "None");
    }

    // Apply Steering
    public void ApplySteering(float targetAngle)
    {
        // Debug.Log($"currentSteeringAngle : {currentSteeringAngle}, targetAngle : {targetAngle}");
        currentSteeringAngle = Mathf.Lerp(currentSteeringAngle, targetAngle, Time.deltaTime * 2f);
        // currentSteeringAngle = targetAngle;
        // Debug.Log($"After clamp currentSteeringAngle : {currentSteeringAngle}, targetAngle : {targetAngle}");
        FLC.steerAngle = currentSteeringAngle;
        FRC.steerAngle = currentSteeringAngle;
    }

    // Apply Motor Torque
    public void ApplyMotorTorque(float targetTorque)
    {
        // Debug.Log($"currentMotorTorque : {currentMotorTorque}, targetTorque : {targetTorque}");
        currentMotorTorque = Mathf.Lerp(currentMotorTorque, targetTorque, Time.deltaTime * 10f);
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
        FRS.localRotation = Quaternion.Euler(45, 0, 0);
        L1S.localRotation = Quaternion.Euler(45, -15, 0);
        L2S.localRotation = Quaternion.Euler(8, -35, 0);
        L3S.localRotation = Quaternion.Euler(10, -90, 0);
        R1S.localRotation = Quaternion.Euler(45, 15, 0);
        R2S.localRotation = Quaternion.Euler(8, 35, 0);
        R3S.localRotation = Quaternion.Euler(10, 90, 0);
        Down.localRotation = Quaternion.Euler(90, 0, 0);
    }
}
// GeneticAlgorithm.cs (Refactored with dynamic gene length, motorTorque and steeringAngle evolved separately)
using System.Collections.Generic;
using UnityEngine;

public class GeneticAlgorithm : MonoBehaviour
{
    // === Config ===
    public int populationSize = 50;
    public int initialGeneLength = 400;
    public float mutationRate = 0.01f;
    public float crossoverRate = 0.7f;
    public int generations = 10000;
    public bool dynamicGeneLength = true;
    public bool useSegmentCrossover = false;
    [SerializeField] private GameObject robotPrefab;

    // === Populations ===
    private List<List<float>> torquePopulation;
    private List<List<float>> steeringPopulation;

    // === Fitness & Control ===
    private List<float> torqueFitnessScores;
    private List<float> steeringFitnessScores;
    private List<RobotController> robotInstances;
    private List<bool> activeIndividuals;
    private int currentStep = 0;
    private int currentGeneration = 0;
    private int currentGeneLength;
    private int freezeIndexTorque = 0;
    private int freezeIndexSteering = 0;
    private bool isCoolDown = false;
    private int maxCoolDownSteps = 500;
    private int coolDownStep = 0;

    private List<float> possibleValues = new List<float>();

    private void Start()
    {
        currentGeneLength = initialGeneLength;

        AudioListener[] listeners = FindObjectsOfType<AudioListener>();
        foreach (AudioListener listener in listeners)
        {
            if (listener != GetComponent<AudioListener>())
            {
                listener.enabled = false;
            }
        }

        InitializePossibleValues();
        InitializePopulation();
        InitializeRobots();
    }

    void InitializePossibleValues()
    {
        for (float v = -1f; v <= 1f; v += 0.04f)
            possibleValues.Add((float)System.Math.Round(v, 2));
    }

    void InitializePopulation()
    {
        torquePopulation = new List<List<float>>();
        steeringPopulation = new List<List<float>>();
        torqueFitnessScores = new List<float>(new float[populationSize]);
        steeringFitnessScores = new List<float>(new float[populationSize]);
        activeIndividuals = new List<bool>(new bool[populationSize]);

        for (int i = 0; i < populationSize; i++)
        {
            torquePopulation.Add(CreateGeneSequence(currentGeneLength, (possibleValues.Count / 2) - 1, (int)(possibleValues.Count * 0.75)));
            steeringPopulation.Add(CreateGeneSequence(currentGeneLength));
            activeIndividuals[i] = true;
        }
    }

    List<float> CreateGeneSequence(int len, int start = 0, int end = -1)
    {
        List<float> seq = new List<float>();
        end = end < 0 ? possibleValues.Count : end;
        for (int i = 0; i < len; i++)
            seq.Add(possibleValues[Random.Range(start, end)]);
        return seq;
    }

    void InitializeRobots()
    {
        if (robotInstances != null)
        {
            foreach (var r in robotInstances)
                if (r != null) Destroy(r.gameObject);
        }

        robotInstances = new List<RobotController>();

        for (int i = 0; i < populationSize; i++)
        {
            GameObject obj = Instantiate(robotPrefab, new Vector3(195.6539f, 0.6679955f, -105f), Quaternion.Euler(0f, 180f, 0f));
            obj.layer = LayerMask.NameToLayer("Robot");
            RobotController rc = obj.GetComponent<RobotController>();
            rc.InitializeForGA(this, i);

            List<Vector2> combined = new List<Vector2>();
            for (int j = 0; j < currentGeneLength; j++)
                combined.Add(new Vector2(torquePopulation[i][j], steeringPopulation[i][j]));

            rc.SetIndividual(combined);
            robotInstances.Add(rc);
        }
    }

    void FixedUpdate()
    {
        if (isCoolDown)
        {
            coolDownStep++;
            if (coolDownStep > maxCoolDownSteps)
            {
                isCoolDown = false;
                coolDownStep = 0;
            }
            // Debug.LogWarning($"Cooling down, step: {coolDownStep}.");
            return;
        }

        if (currentGeneration >= generations) return;

        if (currentStep < currentGeneLength || !AllIndividualsDone())
        {
            for (int i = 0; i < populationSize; i++)
            {
                if (activeIndividuals[i] && currentStep < torquePopulation[i].Count)
                {
                    float torque = torquePopulation[i][currentStep] * 400f;
                    float steer = steeringPopulation[i][currentStep] * 90f;
                    robotInstances[i].ManualApplyControl(torque, steer);
                }
                else if (activeIndividuals[i] && dynamicGeneLength)
                {
                    ExtendIndividual(i, robotInstances[i].GetRoad());
                    float torque = torquePopulation[i][currentStep] * 400f;
                    float steer = steeringPopulation[i][currentStep] * 90f;
                    robotInstances[i].ManualApplyControl(torque, steer);
                }
            }
            currentStep++;

            // ✅ Evaluate fitness during simulation
            for (int i = 0; i < populationSize; i++)
            {
                if (activeIndividuals[i])
                {
                    robotInstances[i].UpdateFitness(currentStep > 1000);
                }
            }
        }
        else if (!activeIndividuals.Contains(true))
        {
            EvolveTorquePopulation();
            EvolveSteeringPopulation();
            isCoolDown = true;
            currentGeneration++;
            currentStep = 0;
            ResetGeneration();
        }
    }

    private bool AllIndividualsDone()
    {
        foreach (bool active in activeIndividuals)
        {
            if (active) return false;
        }
        return true;
    }

    void ExtendIndividual(int index, int turn = 0)
    {
        // int end = turn != 0 ? (int)(possibleValues.Count * 2 / 3) : possibleValues.Count;
        float t = possibleValues[Random.Range((int)(possibleValues.Count/2),possibleValues.Count)];

        int val1 = turn > 0 ? possibleValues.Count * 2 / 3 : 0;
        int val2 = turn < 0 ? possibleValues.Count / 3 : possibleValues.Count;
        float s = possibleValues[Random.Range(val1, val2)];

        Debug.Log($"ExtendIndividual steer: {s}, turn: {turn}");
        torquePopulation[index].Add(t);
        steeringPopulation[index].Add(s);
        currentGeneLength = Mathf.Max(currentGeneLength, torquePopulation[index].Count);
    }

    public void UpdateFitness(int index, float torqueFit, float steerFit, bool done)
    {
        if (done)
        {
            torqueFitnessScores[index] = torqueFit;
            steeringFitnessScores[index] = steerFit;
            activeIndividuals[index] = false;
        }
    }

    void ResetGeneration()
    {
        activeIndividuals = new List<bool>(new bool[populationSize]);
        for (int i = 0; i < populationSize; i++)
            activeIndividuals[i] = true;
        InitializeRobots();
    }

    void EvolveTorquePopulation()
    {
        float best = Max(torqueFitnessScores);
        float avg = Average(torqueFitnessScores);

        Debug.Log($"Torque best: {best}, avg: {avg}, generation: {currentGeneration}, geneLength: {currentGeneLength}");

        // if (avg >= 0.8f * best)
        //     freezeIndexTorque = Mathf.Min(freezeIndexTorque + currentGeneLength / 10, (int)(currentGeneLength / 3f));

        torquePopulation = CreateNewPopulation(torquePopulation, torqueFitnessScores, freezeIndexTorque);
    }

    void EvolveSteeringPopulation()
    {
        float best = Max(steeringFitnessScores);
        float avg = Average(steeringFitnessScores);

        Debug.Log($"Steering best: {best}, avg: {avg}, generation: {currentGeneration}, geneLength: {currentGeneLength}");

        // if (avg >= 0.8f * best)
        //     freezeIndexSteering = Mathf.Min(freezeIndexSteering + currentGeneLength / 10, (int)(currentGeneLength / 3));

        steeringPopulation = CreateNewPopulation(steeringPopulation, steeringFitnessScores, freezeIndexSteering);
    }

    List<List<float>> CreateNewPopulation(List<List<float>> oldPop, List<float> scores, int freezeIndex)
    {
        // 📊 Step 2: Sort individuals based on fitness scores
        List<int> sorted = GetSortedIndices(scores);
        List<List<float>> newPop = new List<List<float>>();

        // 🔁 Step 1: Normalize all gene lengths if dynamic gene length is enabled
        if (dynamicGeneLength)
        {
            for (int i = 0; i < oldPop.Count; i++)
            {
                ExtendToLength(oldPop[i], currentGeneLength, oldPop[sorted[0]]);
            }
        }

        int eliteCount = Mathf.Max(1, populationSize / 10);
        int poolSize = populationSize;

        // 🏅 Step 3: Elitism - carry top individuals unchanged
        for (int i = 0; i < eliteCount; i++)
        {
            newPop.Add(new List<float>(oldPop[sorted[i]]));
        }

        // 🧪 Step 4: Selection pool (best N individuals)
        List<List<float>> pool = new List<List<float>>();
        for (int i = 0; i < poolSize; i++)
        {
            pool.Add(oldPop[sorted[i]]);
        }

        // 🔀 Step 5: Generate offspring via crossover and mutation
        while (newPop.Count < populationSize)
        {
            var p1 = pool[Random.Range(0, poolSize)];
            var p2 = pool[Random.Range(0, poolSize)];

            Crossover(p1, p2, out var c1, out var c2, freezeIndex);
            Mutate(c1, freezeIndex);
            Mutate(c2, freezeIndex);

            if (dynamicGeneLength)
            {
                ExtendToLength(c1, currentGeneLength, oldPop[sorted[0]]); // extend using best individual
                ExtendToLength(c2, currentGeneLength, oldPop[sorted[0]]);
            }

            newPop.Add(c1);
            if (newPop.Count < populationSize) newPop.Add(c2);
        }

        return newPop;
    }


    void ExtendToLength(List<float> individual, int targetLength, List<float> fallbackSource)
    {
        int currentLength = individual.Count;

        for (int i = currentLength; i < targetLength; i++)
        {
            float gene;
            if (i < fallbackSource.Count)
                gene = fallbackSource[i];
            else
                gene = possibleValues[Random.Range(0, possibleValues.Count)]; // fallback if source too short

            individual.Add(gene);
        }
    }


    void Crossover(List<float> p1, List<float> p2, out List<float> c1, out List<float> c2, int freezeIdx)
    {
        c1 = new List<float>(p1);
        c2 = new List<float>(p2);

        if (useSegmentCrossover && p1.Count > freezeIdx + 2)
        {
            int pt1 = Random.Range(freezeIdx + 1, p1.Count - 2);
            int pt2 = Random.Range(pt1, p1.Count - 1);
            for (int i = pt1; i <= pt2; i++)
            {
                float temp = c1[i];
                c1[i] = c2[i];
                c2[i] = temp;
            }
        }
        else
        {
            for (int i = freezeIdx; i < p1.Count; i++)
            {
                if (Random.value < crossoverRate)
                {
                    float temp = c1[i];
                    c1[i] = c2[i];
                    c2[i] = temp;
                }
            }
        }
    }

    void Mutate(List<float> individual, int freezeIdx)
    {
        for (int i = freezeIdx; i < individual.Count; i++)
        {
            float dynamicRate = mutationRate;

            // Higher mutation rate for tail
            if (i > (2 * individual.Count / 5))
                dynamicRate *= 10f;

            if (Random.value < dynamicRate)
                individual[i] = Mathf.Clamp(individual[i] + Random.Range(-0.2f, 0.2f), -1f, 1f);
        }
    }

    float Max(List<float> values) => Mathf.Max(values.ToArray());
    float Average(List<float> values)
    {
        float total = 0f;
        foreach (var v in values) total += v;
        return total / values.Count;
    }

    List<int> GetSortedIndices(List<float> scores)
    {
        List<int> idx = new List<int>();
        for (int i = 0; i < scores.Count; i++) idx.Add(i);
        idx.Sort((a, b) => scores[b].CompareTo(scores[a]));
        return idx;
    }
}

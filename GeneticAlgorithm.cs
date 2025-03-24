using System.Collections.Generic;
using UnityEngine;
using System.IO;
using SimpleJSON;

public class GeneticAlgorithm : MonoBehaviour
{
    // Configuration Parameters
    public int populationSize = 1;
    public int initialGeneLength = 400; // Starting gene length
    public float mutationRate = 0.02f;
    public float crossoverRate = 0.7f;
    public int generations = 10000;

    [SerializeField] private GameObject robotPrefab;

    // Runtime Data
    private List<List<Vector2>> population;
    private List<float> fitnessScores;
    private List<RobotController> robotInstances;
    private List<bool> activeIndividuals;
    private int currentStep = 0;
    private int currentGeneration = 0;
    private int currentGeneLength; // Tracks the current maximum gene length
    private string saveFilePath;
    public bool dynamicGeneLength = true;
    private bool isCoolDown = false;
    private int maxCoolDownSteps = 500;
    private int coolDownStep = 0;
    private int freezeIndex = 0; // Index up to which genes are frozen

    // Initialization
    private void Start()
    {
        QualitySettings.SetQualityLevel(0, true); // Optimize by setting lowest quality
        currentGeneLength = initialGeneLength;
        saveFilePath = Application.persistentDataPath + "/GA_PopulationData.json";
        population = new List<List<Vector2>>();

        AudioListener[] listeners = FindObjectsOfType<AudioListener>();
        foreach (AudioListener listener in listeners)
        {
            if (listener != GetComponent<AudioListener>())
            {
                listener.enabled = false;
            }
        }

        if (LoadPopulationFromFile())
        {
            Debug.Log("✅ Loaded saved population successfully!");
        }
        else
        {
            InitializePopulation();
        }

        InitializeRobots();
    }

    // Cleanup
    private void OnApplicationQuit()
    {
        // SavePopulationToFile();
    }

    // Save Population to File
    private void SavePopulationToFile()
    {
        Debug.Log("💾 Saving population data...");
        string json = "{\n" +
                      $"    \"currentGeneration\": {currentGeneration},\n" +
                      $"    \"currentGeneLength\": {currentGeneLength},\n" +
                      "    \"serializedPopulation\": [\n";

        for (int i = 0; i < population.Count; i++)
        {
            json += "        [\n";
            for (int j = 0; j < population[i].Count; j++)
            {
                Vector2 gene = population[i][j];
                json += $"            [{gene.x}, {gene.y}]";
                if (j < population[i].Count - 1) json += ",";
                json += "\n";
            }
            json += "        ]";
            if (i < population.Count - 1) json += ",";
            json += "\n";
        }
        json += "    ]\n}";

        File.WriteAllText(saveFilePath, json);
        Debug.Log("✅ Population successfully saved to: " + saveFilePath);
    }

    // Load Population from File
    private bool LoadPopulationFromFile()
    {
        if (File.Exists(saveFilePath))
        {
            Debug.Log("📂 Loading population data from file...");
            string json = File.ReadAllText(saveFilePath);
            JSONNode jsonObject = JSON.Parse(json);

            if (jsonObject == null)
            {
                Debug.LogError("❌ Failed to parse JSON file.");
                return false;
            }

            currentGeneration = jsonObject["currentGeneration"].AsInt;
            currentGeneLength = jsonObject["currentGeneLength"].AsInt;
            population = new List<List<Vector2>>();
            JSONNode serializedPopulation = jsonObject["serializedPopulation"];

            if (serializedPopulation == null || serializedPopulation.Count == 0)
            {
                Debug.LogError("❌ Population file is empty.");
                return false;
            }

            foreach (JSONArray individual in serializedPopulation.AsArray)
            {
                List<Vector2> newIndividual = new List<Vector2>();
                foreach (JSONArray gene in individual.AsArray)
                {
                    if (gene.Count < 2) continue;
                    float x = gene[0].AsFloat;
                    float y = gene[1].AsFloat;
                    newIndividual.Add(new Vector2(x, y));
                }
                population.Add(newIndividual);
            }

            fitnessScores = new List<float>(new float[populationSize]);
            activeIndividuals = new List<bool>(new bool[populationSize]);
            for (int i = 0; i < populationSize; i++)
            {
                activeIndividuals[i] = true;
            }
            Debug.Log($"✅ Population loaded. Size: {population.Count}");
            return true;
        }
        Debug.LogWarning("⚠ No saved population found. Starting new.");
        return false;
    }

    // Initialize New Population
    private void InitializePopulation()
    {
        for (int i = 0; i < populationSize; i++)
        {
            population.Add(CreateIndividual(currentGeneLength));
        }
        fitnessScores = new List<float>(new float[populationSize]);
        activeIndividuals = new List<bool>(new bool[populationSize]);
        for (int i = 0; i < populationSize; i++)
        {
            activeIndividuals[i] = true;
        }
    }

    // Create a Random Individual
    private List<Vector2> CreateIndividual(int length)
    {
        List<Vector2> individual = new List<Vector2>();
        for (int j = 0; j < length; j++)
        {
            individual.Add(new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f)));
        }
        return individual;
    }

    // Instantiate Robots
    private void InitializeRobots()
    {
        // Destroy existing robots if any
        if (robotInstances != null)
        {
            foreach (var robot in robotInstances)
            {
                if (robot != null) Destroy(robot.gameObject);
            }
        }

        robotInstances = new List<RobotController>();

        for (int i = 0; i < populationSize; i++)
        {
            GameObject robotObj = Instantiate(robotPrefab, GetSpawnPosition(i), Quaternion.Euler(0f, 180f, 0f));
            robotObj.layer = LayerMask.NameToLayer("Robot");

            RobotController robot = robotObj.GetComponent<RobotController>();
            robot.InitializeForGA(this, i);
            robot.SetIndividual(population[i]); // 👈 assign genome immediately

            robotInstances.Add(robot);

            // Optional: optimize rendering
            if (i < 0)
            {
                Renderer[] renderers = robotObj.GetComponentsInChildren<Renderer>();
                foreach (Renderer renderer in renderers)
                {
                    renderer.enabled = false;
                }
                robot.shouldRender = false;
            }
        }
    }

    // Define Spawn Position
    private Vector3 GetSpawnPosition(int index)
    {
        return new Vector3(195.6539f + index * 2f, 0.6679955f, 192.1293f);
    }

    // Main Simulation Loop
    private void FixedUpdate()
    {
        if (isCoolDown)
        {
            coolDownStep++;
            if (coolDownStep > maxCoolDownSteps)
            {
                isCoolDown = false;
                coolDownStep = 0;
            }
            Debug.LogWarning($"Cooling down, step: {coolDownStep}.");
            return;
        }
        if (population == null || population.Count == 0)
        {
            Debug.LogWarning("⚠ Population not initialized.");
            return;
        }

        if (currentGeneration >= generations) return;

        if (currentStep < currentGeneLength || !AllIndividualsDone())
        {
            for (int i = 0; i < populationSize; i++)
            {
                if (activeIndividuals[i] && currentStep < population[i].Count)
                {
                    float motorTorque = population[i][currentStep].x * 400f;
                    float steeringAngle = population[i][currentStep].y * 60f;
                    robotInstances[i].ManualApplyControl(motorTorque, steeringAngle);
                }
                else if (activeIndividuals[i])
                {
                    if (dynamicGeneLength)
                    {
                        ExtendIndividual(i, robotInstances[i].isOnTurn());
                    }
                }
            }
            currentStep++;

            for (int i = 0; i < populationSize; i++)
            {
                if (activeIndividuals[i])
                {
                    robotInstances[i].UpdateFitness(currentStep > 1000);
                }
            }
        }
        else
        {
            // Debug.Log($"Generation {currentGeneration} ended. Best Fitness: {GetBestFitness()}, Gene Length: {currentGeneLength}");
            EvolvePopulation();
            ResetGeneration();
            isCoolDown = true;
            currentGeneration++;
            if (currentGeneration == generations)
            {
                SaveBestIndividual();
            }

        }
    }

    // Update Individual Fitness
    public void UpdateFitness(int individualIndex, float fitness, bool isDone)
    {
        if (isDone)
        {
            fitnessScores[individualIndex] = fitness;
            activeIndividuals[individualIndex] = false;
        }
    }

    // Check if All Individuals Are Done
    private bool AllIndividualsDone()
    {
        return !activeIndividuals.Contains(true);
    }

    // Extend an Individual’s Gene Sequence
    private void ExtendIndividual(int index, bool isOnTurn)
    {
        population[index].Add(new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f)));
        currentGeneLength = Mathf.Max(currentGeneLength, population[index].Count);
    }

    // Evolve the Population
    private void EvolvePopulation()
    {
        // Step 1: Sort population by fitness (descending)
        List<int> sortedIndices = new List<int>();
        for (int i = 0; i < populationSize; i++) sortedIndices.Add(i);
        sortedIndices.Sort((a, b) => fitnessScores[b].CompareTo(fitnessScores[a]));
        Debug.Log($"sorted first: {fitnessScores[sortedIndices[0]]}.");

        // Step 2: Calculate best and average fitness
        float bestFitness = fitnessScores[sortedIndices[0]];
        float totalFitness = 0f;
        foreach (float fitness in fitnessScores)
            totalFitness += fitness;
        float avgFitness = totalFitness / populationSize;

        // Step 3: Freeze more genes if condition met
        if (avgFitness >= 0.8f * bestFitness && freezeIndex < currentGeneLength - 1)
        {
            int freezeStep = currentGeneLength / 10;
            int targetFreeze = freezeIndex + freezeStep;

            // ❄️ Cap freeze at 50% of current gene length
            freezeIndex = Mathf.Min(targetFreeze, currentGeneLength / 2);

            Debug.Log($"🧊 Freezing genes from 0 to {freezeIndex - 1}");
        }

        // // Step 4: Update current gene length to max observed
        // int maxGeneLength = 0;
        // foreach (var individual in population)
        //     maxGeneLength = Mathf.Max(maxGeneLength, individual.Count);
        // currentGeneLength = maxGeneLength;

        Debug.Log($"Generation {currentGeneration} ended. Best Fitness: {bestFitness:F2}, Avg Fitness: {avgFitness:F2}, Gene Length: {currentGeneLength}");

        // Step 5: Elitism - keep top 10%
        int eliteCount = Mathf.Max(1, populationSize / 10);
        List<List<Vector2>> newPopulation = new List<List<Vector2>>();
        for (int i = 0; i < eliteCount; i++)
        {
            newPopulation.Add(new List<Vector2>(population[sortedIndices[i]]));
        }

        // Step 6: Create breeding pool from top 50%
        int breedingPoolSize = populationSize / 2;
        List<List<Vector2>> breedingPool = new List<List<Vector2>>();
        for (int i = 0; i < breedingPoolSize; i++)
        {
            breedingPool.Add(population[sortedIndices[i]]);
        }

        // Step 7: Generate rest of the population
        while (newPopulation.Count < populationSize)
        {
            var parent1 = breedingPool[Random.Range(0, breedingPoolSize)];
            var parent2 = breedingPool[Random.Range(0, breedingPoolSize)];
            Crossover(parent1, parent2, out var child1, out var child2);
            Mutate(child1);
            Mutate(child2);
            if (dynamicGeneLength)
            {
                ExtendToLength(child1, currentGeneLength);
                ExtendToLength(child2, currentGeneLength);
            }
            newPopulation.Add(child1);
            if (newPopulation.Count < populationSize)
                newPopulation.Add(child2);
        }

        population = newPopulation;

        // Step 8: Assign new individuals to robots
        for (int i = 0; i < populationSize; i++)
        {
            robotInstances[i].SetIndividual(population[i]);
        }
    }



    // Extend Individual to Target Length
    private void ExtendToLength(List<Vector2> individual, int targetLength)
    {
        while (individual.Count < targetLength)
        {
            individual.Add(new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f)));
        }
    }

    // Reset for Next Generation
    private void ResetGeneration()
    {
        currentStep = 0;
        activeIndividuals = new List<bool>(new bool[populationSize]);

        for (int i = 0; i < populationSize; i++)
            activeIndividuals[i] = true;

        InitializeRobots(); // 🔁 Destroy old, spawn new, reset fresh
    }

    // Perform Crossover
    private void Crossover(List<Vector2> parent1, List<Vector2> parent2, out List<Vector2> child1, out List<Vector2> child2)
    {
        child1 = new List<Vector2>(parent1);
        child2 = new List<Vector2>(parent2);

        int maxLength = Mathf.Min(parent1.Count, parent2.Count);
        if (Random.Range(0f, 1f) < crossoverRate && maxLength > freezeIndex + 2)
        {
            int point1 = Random.Range(freezeIndex + 1, maxLength - 2);
            int point2 = Random.Range(point1, maxLength - 1);
            for (int i = point1; i < point2; i++)
            {
                child1[i] = parent2[i];
                child2[i] = parent1[i];
            }
        }
    }


    // Apply Mutation
    private void Mutate(List<Vector2> individual)
    {
        for (int i = freezeIndex; i < individual.Count; i++) // 🔐 only mutate beyond freezeIndex
        {
            if (Random.Range(0f, 1f) < mutationRate)
            {
                individual[i] = new Vector2(
                    Mathf.Clamp(individual[i].x + Random.Range(-0.2f, 0.2f), -1f, 1f),
                    Mathf.Clamp(individual[i].y + Random.Range(-0.2f, 0.2f), -1f, 1f)
                );
            }
        }
    }

    // Get Best Fitness Score
    private float GetBestFitness()
    {
        return Mathf.Max(fitnessScores.ToArray());
    }

    // Save Best Individual
    private void SaveBestIndividual()
    {
        string path = Application.dataPath + "/GA_TrainingData.csv";
        using (StreamWriter writer = new StreamWriter(path))
        {
            writer.WriteLine("MotorTorque,SteeringAngle");
            foreach (Vector2 gene in population[0])
            {
                writer.WriteLine($"{gene.x},{gene.y}");
            }
        }
        Debug.Log("Best individual saved to " + path);
    }
}
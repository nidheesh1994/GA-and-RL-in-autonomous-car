using System.Collections.Generic;
using UnityEngine;
using System.IO;
using SimpleJSON;
public class GeneticAlgorithm : MonoBehaviour
{
    public int populationSize = 50;
    public int initialGeneLength = 100; // Starting gene length
    public float mutationRate = 0.01f;
    public float crossoverRate = 0.7f;
    public int generations = 10000;


    [SerializeField] private GameObject robotPrefab;

    private List<List<Vector2>> population;
    private List<float> fitnessScores;
    private List<RobotController> robotInstances;
    private List<bool> activeIndividuals;
    private int currentStep = 0;
    private int currentGeneration = 0;
    private int currentGeneLength; // Tracks the current maximum gene length

    private string saveFilePath;

    private void Start()
    {
        QualitySettings.SetQualityLevel(0, true);
        currentGeneLength = initialGeneLength; // Initialize to starting value
        saveFilePath = Application.persistentDataPath + "/GA_PopulationData.json";
        population = new List<List<Vector2>>();

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

    private void OnApplicationQuit()
    {
        SavePopulationToFile();
    }

    private void SavePopulationToFile()
    {
        Debug.Log("💾 Saving population data...");

        // Start JSON structure
        string json = "{\n";
        json += $"    \"currentGeneration\": {currentGeneration},\n";
        json += $"    \"currentGeneLength\": {currentGeneLength},\n";
        json += "    \"serializedPopulation\": [\n";

        // Loop through each individual
        for (int i = 0; i < population.Count; i++)
        {
            json += "        [\n";

            // Loop through each gene (Vector2)
            for (int j = 0; j < population[i].Count; j++)
            {
                Vector2 gene = population[i][j];
                json += $"            [{gene.x}, {gene.y}]";

                // Add comma unless it's the last element
                if (j < population[i].Count - 1) json += ",";
                json += "\n";
            }

            json += "        ]";

            // Add comma unless it's the last individual
            if (i < population.Count - 1) json += ",";
            json += "\n";
        }

        json += "    ]\n}";

        // Write to file
        File.WriteAllText(saveFilePath, json);
        Debug.Log("✅ Population successfully saved to: " + saveFilePath);
    }





    private bool LoadPopulationFromFile()
    {
        if (File.Exists(saveFilePath))
        {
            Debug.Log("📂 Loading population data from file...");

            string json = File.ReadAllText(saveFilePath);
            JSONNode jsonObject = JSON.Parse(json);  // ✅ Fix: Use `JSONNode`

            if (jsonObject == null)
            {
                Debug.LogError("❌ Failed to parse JSON file. The file may be corrupted.");
                return false;
            }

            currentGeneration = jsonObject["currentGeneration"].AsInt;
            currentGeneLength = jsonObject["currentGeneLength"].AsInt;

            population = new List<List<Vector2>>();

            JSONNode serializedPopulation = jsonObject["serializedPopulation"];

            if (serializedPopulation == null || serializedPopulation.Count == 0)
            {
                Debug.LogError("❌ Error: Population file exists but is empty.");
                return false;
            }

            foreach (JSONArray individual in serializedPopulation.AsArray)
            {
                List<Vector2> newIndividual = new List<Vector2>();

                foreach (JSONArray gene in individual.AsArray)
                {
                    if (gene.Count < 2) continue; // 🔥 Fix: Ensure gene has 2 values before accessing
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

            Debug.Log($"✅ Population successfully loaded. Population Size: {population.Count}");
            return true;
        }
        else
        {
            Debug.LogWarning("⚠ No saved population file found. Starting new.");
            return false;
        }
    }






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

    private List<Vector2> CreateIndividual(int length)
    {
        List<Vector2> individual = new List<Vector2>();
        for (int j = 0; j < length; j++)
        {
            individual.Add(new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f)));
        }
        return individual;
    }

    private void InitializeRobots()
    {
        robotInstances = new List<RobotController>();
        for (int i = 0; i < populationSize; i++)
        {
            GameObject robotObj = Instantiate(robotPrefab, GetSpawnPosition(i), Quaternion.Euler(0f, 180f, 0f));
            robotObj.layer = LayerMask.NameToLayer("Robot");
            RobotController robot = robotObj.GetComponent<RobotController>();
            robot.InitializeForGA(this, i);
            robotInstances.Add(robot);
        }
    }

    private Vector3 GetSpawnPosition(int index)
    {
        return new Vector3(195.6539f + index * 2f, 0.6679955f, 192.1293f);
    }

    private void FixedUpdate()
    {
        if (population == null || population.Count == 0)
        {
            Debug.LogWarning("⚠ Population is not initialized. Skipping FixedUpdate.");
            return;
        }

        if (currentGeneration >= generations) return;

        if (currentStep < currentGeneLength || !AllIndividualsDone())
        {
            // Debug.Log($"Current step: {currentStep}, currentGeneration: {currentGeneration}, currentGeneLength {currentGeneLength}");
            // Step active robots forward in parallel
            for (int i = 0; i < populationSize; i++)
            {
                if (activeIndividuals[i])
                {
                    if (currentStep < population[i].Count) // Ensure we don’t exceed individual gene length
                    {
                        float motorTorque = population[i][currentStep].x * 400f;
                        float steeringAngle = population[i][currentStep].y * 60f;
                        robotInstances[i].ManualApplyControl(motorTorque, steeringAngle);
                    }
                    else
                    {
                        // Car is still active but out of genes; extend its sequence
                        ExtendIndividual(i, robotInstances[i].isOnTurn());
                    }
                }
            }
            currentStep++;

            // Update fitness and check completion
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
            // End of generation
            Debug.Log($"Generation {currentGeneration} ended. Best Fitness: {GetBestFitness()}, Gene Length: {currentGeneLength}");
            EvolvePopulation();
            ResetGeneration();
            currentGeneration++;
            if (currentGeneration == generations)
            {
                SaveBestIndividual();
            }
        }
    }

    public void UpdateFitness(int individualIndex, float fitness, bool isDone)
    {
        if (isDone)
        {
            fitnessScores[individualIndex] = fitness;
            activeIndividuals[individualIndex] = false;
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

    private void ExtendIndividual(int index, bool isOnTurn)
    {
        if (Random.value < 0.1f) // 10% chance for a fully random gene
        {
            population[index].Add(new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f)));
        }
        else
        {
            Vector2 lastGene = population[index][population[index].Count - 1];
            float newTorque = lastGene.x + Random.Range(-0.1f, 0.1f);
            float newSteering = lastGene.y + Random.Range(-0.1f, 0.1f);

            // Bias torque and steering on turns
            if (isOnTurn)
            {
                newTorque = Mathf.Clamp(newTorque, -1f, 1f); // Allow braking/reversing
                newSteering = Mathf.Clamp(newSteering + Random.Range(-0.2f, 0.2f), -1f, 1f); // Larger steering variance
            }
            else
            {
                newTorque = Mathf.Clamp(newTorque, 0f, 1f); // Forward movement only
                newSteering = Mathf.Clamp(newSteering, -1f, 1f);
            }

            population[index].Add(new Vector2(newTorque, newSteering));
        }
        currentGeneLength = Mathf.Max(currentGeneLength, population[index].Count);
    }

    private void EvolvePopulation()
    {
        // 🔥 Step 1: Sort Population by Fitness (Descending Order)
        List<int> sortedIndices = new List<int>();
        for (int i = 0; i < populationSize; i++) sortedIndices.Add(i);

        sortedIndices.Sort((a, b) => fitnessScores[b].CompareTo(fitnessScores[a])); // Sort by highest fitness

        List<List<Vector2>> newPopulation = new List<List<Vector2>>();

        // 🔥 Step 2: Select the top 50% of the population
        int eliteSize = populationSize / 2;
        List<List<Vector2>> eliteIndividuals = new List<List<Vector2>>();
        for (int i = 0; i < eliteSize; i++)
        {
            eliteIndividuals.Add(population[sortedIndices[i]]);
        }

        // 🔥 Step 3: Generate the Next Generation
        for (int i = 0; i < populationSize; i += 2)
        {
            // Select parents from the elite pool
            var parent1 = eliteIndividuals[Random.Range(0, eliteSize)];
            var parent2 = eliteIndividuals[Random.Range(0, eliteSize)];

            // Perform crossover
            Crossover(parent1, parent2, out var child1, out var child2);

            // Apply mutation
            Mutate(child1);
            Mutate(child2);

            // Ensure children match the maximum gene length
            ExtendToLength(child1, currentGeneLength);
            ExtendToLength(child2, currentGeneLength);

            newPopulation.Add(child1);
            newPopulation.Add(child2);
        }

        // 🔥 Step 4: Update the population with the new generation
        population = newPopulation;

        // 🔥 Step 5: Assign new individuals to robots
        for (int i = 0; i < populationSize; i++)
        {
            robotInstances[i].SetIndividual(population[i]);
        }
    }


    private void ExtendToLength(List<Vector2> individual, int targetLength)
    {
        while (individual.Count < targetLength)
        {
            individual.Add(new Vector2(Random.Range(-1f, 1f), Random.Range(-1f, 1f)));
        }
    }

    private void ResetGeneration()
    {
        currentStep = 0;
        for (int i = 0; i < populationSize; i++)
        {
            activeIndividuals[i] = true;
            robotInstances[i].ManualReset();
        }
    }

    private void Crossover(List<Vector2> parent1, List<Vector2> parent2, out List<Vector2> child1, out List<Vector2> child2)
    {
        child1 = new List<Vector2>(parent1);
        child2 = new List<Vector2>(parent2);

        if (Random.Range(0f, 1f) < crossoverRate)
        {
            int maxLength = Mathf.Min(parent1.Count, parent2.Count); // Use shorter length for crossover
            int point1 = Random.Range(1, maxLength - 2);
            int point2 = Random.Range(point1, maxLength - 1);

            for (int i = point1; i < point2; i++)
            {
                child1[i] = parent2[i];
                child2[i] = parent1[i];
            }
        }
    }

    private void Mutate(List<Vector2> individual)
    {
        for (int i = 0; i < individual.Count; i++)
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

    private List<Vector2> SelectParent()
    {
        int candidate1 = Random.Range(0, populationSize);
        int candidate2 = Random.Range(0, populationSize);
        return fitnessScores[candidate1] > fitnessScores[candidate2] ? population[candidate1] : population[candidate2];
    }

    private float GetBestFitness()
    {
        float bestFitness = float.MinValue;
        foreach (float fitness in fitnessScores)
        {
            if (fitness > bestFitness)
            {
                bestFitness = fitness;
            }
        }
        return bestFitness;
    }

    private void SaveBestIndividual()
    {
        string path = Application.dataPath + "/GA_TrainingData.csv";
        using (System.IO.StreamWriter writer = new System.IO.StreamWriter(path))
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
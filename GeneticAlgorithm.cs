using System.Collections.Generic;
using UnityEngine;

public class GeneticAlgorithm : MonoBehaviour
{
    public int populationSize = 50;
    public int initialGeneLength = 100; // Starting gene length
    public float mutationRate = 0.01f;
    public float crossoverRate = 0.7f;
    public int generations = 100;

    [SerializeField] private GameObject robotPrefab;

    private List<List<Vector2>> population;
    private List<float> fitnessScores;
    private List<RobotController> robotInstances;
    private List<bool> activeIndividuals;
    private int currentStep = 0;
    private int currentGeneration = 0;
    private int currentGeneLength; // Tracks the current maximum gene length

    private void Start()
    {
        QualitySettings.SetQualityLevel(0, true);
        currentGeneLength = initialGeneLength; // Initialize to starting value
        InitializePopulation();
        InitializeRobots();
    }

    private void InitializePopulation()
    {
        population = new List<List<Vector2>>();
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
            individual.Add(new Vector2(Random.Range(0f, 1f), Random.Range(-1f, 1f)));
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
        // Add a new random gene to this individual’s sequence
        population[index].Add(new Vector2(Random.Range(isOnTurn ? -1f : 0, 1f), Random.Range(-1f, 1f)));
        // Update currentGeneLength if this individual’s length exceeds it
        currentGeneLength = Mathf.Max(currentGeneLength, population[index].Count);
    }

    private void EvolvePopulation()
    {
        // Find the maximum gene length in the current population
        int maxGeneLength = currentGeneLength;
        foreach (var individual in population)
        {
            maxGeneLength = Mathf.Max(maxGeneLength, individual.Count);
        }
        currentGeneLength = maxGeneLength; // Update to the maximum observed
        Debug.Log($"Generation: Current genelenght: {currentGeneLength}");
        List<List<Vector2>> newPopulation = new List<List<Vector2>>();
        for (int i = 0; i < populationSize; i += 2)
        {
            var parent1 = SelectParent();
            var parent2 = SelectParent();
            Crossover(parent1, parent2, out var child1, out var child2);
            Mutate(child1);
            Mutate(child2);

            // Ensure children match the maximum gene length
            ExtendToLength(child1, currentGeneLength);
            ExtendToLength(child2, currentGeneLength);

            newPopulation.Add(child1);
            newPopulation.Add(child2);
        }
        population = newPopulation;

        // Update individuals for each robot
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
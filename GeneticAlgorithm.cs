using System.Collections.Generic;
using UnityEngine;

public class GeneticAlgorithm : MonoBehaviour
{
    public int populationSize = 50;
    public int geneLength = 10000;
    public float mutationRate = 0.01f;
    public float crossoverRate = 0.7f;
    public int generations = 100;
    
    [SerializeField] private GameObject robotPrefab;

    private List<List<Vector2>> population;
    private List<float> fitnessScores;
    private List<RobotController> robotInstances;
    private List<bool> activeIndividuals; // Tracks which robots are still active
    private int currentStep = 0;
    private int currentGeneration = 0;


    private void Start()
    {
        InitializePopulation();
        InitializeRobots();
    }

    private void InitializePopulation()
    {
        population = new List<List<Vector2>>();
        for (int i = 0; i < populationSize; i++)
        {
            population.Add(CreateIndividual());
        }
        fitnessScores = new List<float>(new float[populationSize]);
        activeIndividuals = new List<bool>(new bool[populationSize]);
        for (int i = 0; i < populationSize; i++)
        {
            activeIndividuals[i] = true; // All start active
        }
    }

    private List<Vector2> CreateIndividual()
    {
        List<Vector2> individual = new List<Vector2>();
        for (int j = 0; j < geneLength; j++)
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
        if (currentGeneration >= generations) return;

        if (currentStep < geneLength && !AllIndividualsDone())
        {
            // Step active robots forward in parallel
            for (int i = 0; i < populationSize; i++)
            {
                if (activeIndividuals[i])
                {
                    float motorTorque = population[i][currentStep].x * 400f;
                    float steeringAngle = population[i][currentStep].y * 60f;
                    robotInstances[i].ManualApplyControl(motorTorque, steeringAngle);
                }
            }
            currentStep++;

            // Update fitness and check completion
            for (int i = 0; i < populationSize; i++)
            {
                if (activeIndividuals[i])
                {
                    robotInstances[i].UpdateFitness();
                }
            }
        }
        else
        {
            // End of generation
            Debug.Log($"Generation {currentGeneration} ended. Best Fitness: {GetBestFitness()}");
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
            activeIndividuals[individualIndex] = false; // Mark as inactive
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

    private void EvolvePopulation()
    {
        List<List<Vector2>> newPopulation = new List<List<Vector2>>();
        for (int i = 0; i < populationSize; i += 2)
        {
            var parent1 = SelectParent();
            var parent2 = SelectParent();
            Crossover(parent1, parent2, out var child1, out var child2);
            Mutate(child1);
            Mutate(child2);
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

    private void ResetGeneration()
    {
        currentStep = 0;
        for (int i = 0; i < populationSize; i++)
        {
            activeIndividuals[i] = true;
            robotInstances[i].ManualReset();
        }
    }
    

    // Existing methods: Crossover, Mutate, SelectParent, GetBestFitness, SaveBestIndividual
    private void Crossover(List<Vector2> parent1, List<Vector2> parent2, out List<Vector2> child1, out List<Vector2> child2)
    {
        child1 = new List<Vector2>(parent1);
        child2 = new List<Vector2>(parent2);

        if (Random.Range(0f, 1f) < crossoverRate)
        {
            int point1 = Random.Range(1, geneLength - 2);
            int point2 = Random.Range(point1, geneLength - 1);

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
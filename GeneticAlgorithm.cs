using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class GeneticAlgorithm : MonoBehaviour
{
    public int populationSize = 50;
    public int geneLength = 2000;
    public float mutationRate = 0.01f;
    public float crossoverRate = 0.7f;
    public int generations = 100;

    public List<List<Vector2>> population;
    private List<float> fitnessScores;

    private RobotController robotController;

    private void Start()
    {
        robotController = FindObjectOfType<RobotController>();
        InitializePopulation();
        StartCoroutine(RunGA());
    }

    private void InitializePopulation()
    {
        population = new List<List<Vector2>>();
        for (int i = 0; i < populationSize; i++)
        {
            population.Add(CreateIndividual());
        }
        fitnessScores = new List<float>(new float[populationSize]);
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

    public void UpdateFitness(float fitness)
    {
        int currentIndividual = fitnessScores.Count - population.Count;
        if (currentIndividual >= 0 && currentIndividual < fitnessScores.Count)
        {
            fitnessScores[currentIndividual] = fitness;
        }
    }

    private IEnumerator RunGA()
    {
        for (int generation = 0; generation < generations; generation++)
        {
            Debug.Log($"Generation {generation}: starting");

            // Evaluate fitness for each individual in the population
            yield return StartCoroutine(EvaluatePopulation());

            Debug.Log($"Generation {generation}: ended Best Fitness = {GetBestFitness()}");

            EvolvePopulation();

            yield return null;  // Allow Unity to update frames before next generation
        }

        SaveBestIndividual();
    }

    // 🔥🔥 Evaluate Fitness for Each Individual 🔥🔥
    private IEnumerator EvaluatePopulation()
    {
        for (int i = 0; i < populationSize; i++)
        {
            float fitness = 0f;

            // Run each individual's control sequence and get fitness score
            yield return StartCoroutine(RunIndividual(population[i], result => fitness = result));

            Debug.Log($"Fitness score of pupulation {i}: {fitness}");

            fitnessScores[i] = fitness;  // Store the fitness score
        }
    }

    // 🔥🔥 Run Each Individual's Control Sequence 🔥🔥
    private IEnumerator RunIndividual(List<Vector2> individual, System.Action<float> callback)
    {
        float totalReward = 0f;
        robotController.ManualReset();  // Reset the car for each individual

        for (int t = 0; t < geneLength; t++)
        {
            float motorTorque = individual[t].x * 400f;
            float steeringAngle = individual[t].y * 60f;

            robotController.ManualApplyControl(motorTorque, steeringAngle);

            Debug.Log($"Motor Torque: {motorTorque}, Steering Angle: {steeringAngle}, currentStep: {t}");

            // Get reward as fitness score
            float reward = robotController.HandleTrackRewards(motorTorque, steeringAngle);
            totalReward += reward;

            if (robotController.IsOutOfTrack() || robotController.HandleFinalCheckpoint())
            {
                break;  // End the simulation if out of track or finished
            }

            yield return new WaitForFixedUpdate();  // Wait for next physics frame
        }

        callback(totalReward);  // Return fitness score via callback
    }

    private void EvolvePopulation()
    {
         Debug.Log($"Evolving");
        List<List<Vector2>> newPopulation = new List<List<Vector2>>();
        for (int i = 0; i < populationSize; i += 2)
        {
            var parent1 = SelectParent();
            var parent2 = SelectParent();
            Crossover(parent1, parent2, out var child1, out var child2);  // Crossover function defined below
            Mutate(child1);
            Mutate(child2);
            newPopulation.Add(child1);
            newPopulation.Add(child2);
        }
        population = newPopulation;
    }

    // 🔥🔥 Define the Crossover Function 🔥🔥
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
        float bestFitness = 0f;
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

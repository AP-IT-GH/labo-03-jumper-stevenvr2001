using System.Collections.Generic; // Nodig voor List
using UnityEngine;

public class SpawnerScript : MonoBehaviour
{
    // === INSPECTOR VARIABELEN ===
    [Header("Object Prefabs")]
    public GameObject obstaclePrefab;
    public GameObject targetPrefab;

    [Header("Spawner Gebieden")]
    public Transform spawnerXArea; // Moet een Collider hebben
    public Transform spawnerZArea; // Moet een Collider hebben

    [Header("Spawn Timing (Seconden)")]
    public float minSpawnTime = 3.0f;
    public float maxSpawnTime = 5.0f;

    [Header("Obstakel Snelheid")]
    public float minSpeed = 2.0f;
    public float maxSpeed = 5.0f;

    // === PRIVÉ VARIABELEN ===
    private float nextSpawnTime;
    private Collider spawnerXCollider;
    private Collider spawnerZCollider;
    private List<GameObject> spawnedObjects = new List<GameObject>(); // Lijst om gespawnde objecten bij te houden

    private enum SpawnType { Obstakel, Target }

    // === UNITY METHODEN ===

    void Start()
    {
        // Haal colliders op en valideer Inspector variabelen
        if (!ValidateSettings())
        {
            this.enabled = false;
            return;
        }
        ScheduleNextSpawn();
    }

    void Update()
    {
        if (Time.time >= nextSpawnTime)
        {
            SpawnNewObject();
            ScheduleNextSpawn();
        }
    }

    // === PUBLIC METHODEN ===

    // Wordt aangeroepen door de Agent aan het begin van een episode
    public void ResetSpawner()
    {
        // Verwijder alle nog bestaande obstakels/targets
        foreach (GameObject obj in spawnedObjects)
        {
            // Check of het object nog bestaat voor je het probeert te vernietigen
            if (obj != null)
            {
                Destroy(obj);
            }
        }
        spawnedObjects.Clear(); // Maak de lijst leeg

        // Plan de eerste spawn voor de nieuwe episode
        ScheduleNextSpawn();
    }


    // === SPAWN LOGICA ===

    void ScheduleNextSpawn()
    {
        float randomDelay = Random.Range(minSpawnTime, maxSpawnTime);
        nextSpawnTime = Time.time + randomDelay;
    }

    void SpawnNewObject()
    {
        //Kies Spawner & Richting
        bool spawnOnX = (Random.Range(0, 2) == 0);
        Collider chosenSpawnerCollider = spawnOnX ? spawnerXCollider : spawnerZCollider;
        Vector3 moveDirection = spawnOnX ? Vector3.right : Vector3.forward;

        //Kies Type & Prefab
        // 60% kans op obstakel, 40% op target
        float obstacleProbability = 0.6f;
        SpawnType typeToSpawn = (Random.value < obstacleProbability) ? SpawnType.Obstakel : SpawnType.Target;
        GameObject prefabToSpawn = (typeToSpawn == SpawnType.Obstakel) ? obstaclePrefab : targetPrefab;
        string tagToApply = (typeToSpawn == SpawnType.Obstakel) ? "Obstakel" : "Target";

        //Bepaal Spawn Positie
        Vector3 spawnPosition = GetRandomPositionInBounds(chosenSpawnerCollider.bounds);
        spawnPosition.y = prefabToSpawn.transform.position.y; // Gebruik Y van prefab (of stel vaste hoogte in)

        //Instantieer
        GameObject newObject = Instantiate(prefabToSpawn, spawnPosition, Quaternion.identity);
        newObject.tag = tagToApply; // Tag instellen is belangrijk!

        //Initialiseer Beweging
        ObstacleMovement movementScript = newObject.GetComponent<ObstacleMovement>();
        if (movementScript != null)
        {
            float randomSpeed = Random.Range(minSpeed, maxSpeed);
            movementScript.Initialize(randomSpeed, moveDirection);
        }
        else
        {
            Debug.LogError($"Prefab '{prefabToSpawn.name}' mist het ObstacleMovement script!", newObject);
            Destroy(newObject); // Vernietig object als het niet kan bewegen
            return; // Stop verdere uitvoering voor dit object
        }

        //Voeg toe aan lijst voor beheer
        spawnedObjects.Add(newObject);

        // Verwijder null entries uit de lijst (objecten die zichzelf vernietigd hebben)
        spawnedObjects.RemoveAll(item => item == null);
    }

    // === HELPER METHODEN ===

    private Vector3 GetRandomPositionInBounds(Bounds bounds)
    {
        // Houd rekening met de rotatie van de spawner, neem lokale bounds
        float randomX = Random.Range(-bounds.extents.x, bounds.extents.x);
        float randomZ = Random.Range(-bounds.extents.z, bounds.extents.z);
        // Converteer lokaal punt naar wereld positie
        return bounds.center + new Vector3(randomX, 0, randomZ);
    }

    private bool ValidateSettings()
    {
        bool valid = true;
        if (obstaclePrefab == null) { Debug.LogError("Obstacle Prefab niet ingesteld!"); valid = false; }
        if (targetPrefab == null) { Debug.LogError("Target Prefab niet ingesteld!"); valid = false; }
        if (spawnerXArea == null) { Debug.LogError("Spawner X Area niet ingesteld!"); valid = false; }
        else { spawnerXCollider = spawnerXArea.GetComponent<Collider>(); if(spawnerXCollider == null) {Debug.LogError("Spawner X Area mist Collider!"); valid = false;}}
        if (spawnerZArea == null) { Debug.LogError("Spawner Z Area niet ingesteld!"); valid = false; }
        else { spawnerZCollider = spawnerZArea.GetComponent<Collider>(); if(spawnerZCollider == null) {Debug.LogError("Spawner Z Area mist Collider!"); valid = false;}}

        // Controleer of prefabs de nodige scripts/componenten hebben
        if(valid && obstaclePrefab.GetComponent<ObstacleMovement>() == null) { Debug.LogError("Obstacle Prefab mist ObstacleMovement script!"); valid = false; }
        if(valid && targetPrefab.GetComponent<ObstacleMovement>() == null) { Debug.LogError("Target Prefab mist ObstacleMovement script!"); valid = false; }

        return valid;
    }
}
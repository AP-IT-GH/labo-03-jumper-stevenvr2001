# Rapport: Jumper Agent Obstacle Course

**Datum**: 28 april 2025

**S-nummer**: s113187

## Beschrijving

Dit project bevat een Unity-omgeving waarin een ML-Agent (de "Jumper Agent") leert om over bewegende obstakels te springen. De agent moet navigeren op een platform, obstakels ontwijken die vanaf de zijkanten worden gespawnd, en mogelijk targets verzamelen. Het doel is om de agent te trainen met reinforcement learning om zo lang mogelijk te overleven en/of zoveel mogelijk targets te pakken.


## Screenshots


## Features

* Een bestuurbare agent met ML-Agents.
* Spring- en bewegingsmechanismen.
* Dynamisch gespawnde obstakels en targets met variabele snelheid.
* Observatiesysteem dat rekening houdt met de omgeving (grondcontact, positie, snelheid, nabije obstakels).
* Beloningssysteem (+1 voor target, -1 voor obstakel geraakt, -1 voor vallen).
* Mogelijkheid tot handmatige besturing (Heuristic mode).

## Installatie & Setup

Deze sectie beschrijft hoe je de omgeving stap voor stap kunt opbouwen en testen.

### Benodigdheden (Vooraf)

* **Unity Editor:** Versie 6.0.0f1 of compatibel.
* **ML-Agents:** Unity ML-Agents package (bijv. `com.unity.ml-agents`). Installeer dit via de Unity Package Manager.
* **Project Files:** Zorg dat je de scripts (`JumperAgent.cs`, `ObstacleMovement.cs`, `SpawnerScript.cs`) en eventuele bestaande prefabs/materialen in je Unity project hebt.

### 1. Opzetten van Basis Unity Omgeving

Begin met het creëren van de fysieke speelruimte.

1.  **Maak een nieuwe Scene:** Start met een lege 3D scene.
2.  **Maak de TrainingArea:**
    * Creëer een leeg GameObject (Create Empty). Noem dit "TrainingArea".
    * Reset de Transform naar `(0,0,0)`.
    * Plaats starks "Vloer", "Agent" en "Spawner" als kinderen van de "TrainingArea". Dit helpt bij het organiseren en eventueel dupliceren van de omgeving.
3.  **Maak de Vloer:**
    * Creëer een rechthoekig `Plane` GameObject (GameObject -> 3D Object -> Plane). Noem dit bijvoorbeeld "Vloer".
    * Reset de Transform naar `(0,0,0)`.
    * Geef het een geschikte `Material` (optioneel).
    * Voeg een `Collider` toe (standaard aanwezig op Plane).
    * Wijs een specifieke **Layer** toe, bijvoorbeeld "Ground". Deze laag gebruik je later om te detecteren of de agent op de grond staat.
    * dupliceer dit element en draai het 90 graden zodat je een 'kruispunt' krijgt.

    ![Hoe de omgeving er nu zou moeten uitzien](<Schermafbeelding 2025-04-26 190257.jpg>)
4.  **Maak Spawner Gebieden:**
    * Creëer twee lege GameObjects (kleinere rechthoekige planes die je aan de uiteinden van de vloer zet), bijvoorbeeld "SpawnerX" en "SpawnerZ".
    * Plaats ze aan de randen van het speelveld waar de obstakels/targets moeten verschijnen.
    * Voeg aan beide een `Collider` component toe (bijv. `Box Collider`) en stel de grootte zo in dat het de gewenste spawnzone beslaat. Zet `Is Trigger` **uit**. Deze colliders worden door het `SpawnerScript` gebruikt om de spawnpositie te bepalen.
    * maak de tags `SpawnerZ` en `SpawnerX` aan en wijs ze toe aan de juiste spawner

    ![alt text](<Schermafbeelding 2025-04-26 191632.jpg>)
5.  **Maak een Startgebied:**
    * Creëer een leeg GameObject "StartArea" waar de agent moet beginnen. Dit doe je op dezelfde manier als de spawners die je net aanmaakte, alleen wijs je de tag `StartArea` toe.
    
    ![alt text](<Schermafbeelding 2025-04-26 191840.jpg>)


### 2. De Nodige Prefabs Aanmaken (`Obstakel` & `Target`)

Maak de objecten die gespawnd zullen worden uit de spawners.

1.  **Maak het Obstakel:**
    * Creëer een `Cube` GameObject in de scene. Noem het "Obstakel".
    * Geef het een duidelijk `Material` (bijv. rood).
    * Voeg een `Rigidbody` component toe. Zet `Use Gravity` uit (tenzij ze moeten vallen) en vries de rotaties aan (`Freeze Rotation` X, Y, Z) om te voorkomen dat ze kantelen.
    * Zorg dat er een `Collider` op zit (bijv. `Box Collider`).
    * Maak een legg MonoBehaviour script aan genaamd **`ObstacleMovement.cs`** en voeg het toe aan dit GameObject.
    * Stel de **Tag** in op "Obstakel" (rechtsboven in de Inspector, naast Layer). Dit is cruciaal voor de collision detectie in `JumperAgent.cs`.
    * Sleep het "Obstakel" GameObject vanuit de Hierarchy naar je Project venster (bijv. in een `Prefabs` map) om er een **Prefab** van te maken. Verwijder het object daarna uit de scene.
2.  **Maak het Target:**
    * Creëer een ander 3D Object (bijv. een Sphere of Cube). Noem het "Target".
    * Geef het een ander `Material` (bijv. groen).
    * Voeg een `Rigidbody` component toe. Configureer `Use Gravity` en `Freeze Rotation` naar wens.
    * Zorg voor een `Collider`. Zet `Is Trigger` eventueel aan als de agent er doorheen moet kunnen lopen en het alleen een trigger event moet geven (pas dan wel `OnCollisionEnter` aan naar `OnTriggerEnter` in `JumperAgent.cs`). Als het een fysiek object moet zijn dat je raakt, laat `Is Trigger` uit.
    * Voeg het **`ObstacleMovement.cs`** script toe als de targets ook moeten bewegen op dezelfde manier als obstakels. Als ze stil moeten staan, is dit script niet nodig.
    * Stel de **Tag** in op "Target".
    * Maak hier ook een **Prefab** van door het naar het Project venster te slepen en verwijder het uit de scene.

![alt text](<Schermafbeelding 2025-04-26 192544.jpg>) ![alt text](<Schermafbeelding 2025-04-26 192527.jpg>)
### 3. De Spawners Uitwerken (`SpawnerScript.cs` en `ObstacleMovement.cs`)

Nu de prefabs klaar zijn, maken we de logica die ze in de scene brengt en laat bewegen.

**1. Werk `ObstacleMovement.cs` uit (Script voor Prefabs):**

Dit script is verantwoordelijk voor de beweging van de gespawnde objecten (`Obstakel` en `Target`). Zorg ervoor dat dit script **aanwezig is in je project** en **toegevoegd is aan zowel de `Obstakel` als de `Target` prefab**, zoals beschreven in setup stap 2 ("De Nodige Prefabs Aanmaken").

* **Doel:** Het script ontvangt een snelheid en richting van de `SpawnerScript` en beweegt het GameObject met een constante snelheid in die richting. Het zorgt er ook voor dat het object niet kantelt en zichzelf opruimt als het van het speelveld valt.
* **Belangrijkste Functies:**
    * `Initialize(float speed, Vector3 direction)`: Wordt aangeroepen door de spawner om de beweging in te stellen.
    * `FixedUpdate()`: Verplaatst de `Rigidbody` fysiek correct.
    * `Awake()`: Haalt de `Rigidbody` op en stelt constraints in om rotatie te voorkomen.
    * `Update()`: Controleert of het object te laag is gevallen en vernietigt het indien nodig.

* **Code (`ObstacleMovement.cs`):**

    ```csharp
    using UnityEngine;

    // Vereist dat het GameObject een Rigidbody heeft
    [RequireComponent(typeof(Rigidbody))]
    public class ObstacleMovement : MonoBehaviour
    {
        // === PRIVATE VARIABELEN ===
        private float moveSpeed;       // Snelheid (ingesteld door Spawner)
        private Vector3 moveDirection; // Richting (ingesteld door Spawner)
        private Rigidbody rb;          // Referentie naar de Rigidbody

        // === PUBLIEKE GETTERS ===
        // De Agent heeft mogelijk de snelheid en richting nodig voor observaties
        public float CurrentSpeed => moveSpeed;
        public Vector3 MoveDirection => moveDirection;

        // === PUBLIC METHODEN ===

        // Initialiseer snelheid en richting (aangeroepen door Spawner)
        public void Initialize(float speed, Vector3 direction)
        {
            this.moveSpeed = speed;
            this.moveDirection = direction.normalized;
        }

        // === UNITY METHODEN ===

        void Awake()
        {
            rb = GetComponent<Rigidbody>();
            if (rb == null)
            {
                Debug.LogError("ObstacleMovement: Rigidbody component niet gevonden!", this.gameObject);
                this.enabled = false; // Schakel script uit
            }
            // Zorg ervoor dat de Rigidbody niet kantelt of draait
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ;
        }

        void FixedUpdate()
        {
            // Beweeg de Rigidbody indien geïnitialiseerd
            if (moveSpeed > 0 && moveDirection != Vector3.zero && rb != null)
            {
                rb.MovePosition(rb.position + moveDirection * moveSpeed * Time.fixedDeltaTime);
            }
        }

        void Update()
        {
            // Verwijder object als het te ver valt of te ver weg is
            float fallThreshold = -5.0f; // Onder deze Y-waarde wordt het object verwijderd

            if (transform.position.y < fallThreshold)
            {
                Destroy(this.gameObject);
            }
        }
    }
    ```

**2. Maak het `SpawnerScript.cs` script:**

Dit script regelt *wanneer*, *waar* en *wat* er gespawnd wordt.

1.  **Creëer het Script:** Maak een nieuw MonoBehaviour script in je Unity Project (bijv. in de `Scripts` map) en noem het `SpawnerScript`.
2.  **Voeg de Code toe:** Open het script en plak de volgende code erin:

    ```csharp
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
    ```

**3. Wijs `SpawnerScript.cs` toe en Configureer:**

Nu koppelen we het script aan een GameObject in de scene en stellen we het in.

1.  **Maak/Selecteer Spawner GameObject:** Zorg dat je een leeg GameObject "Spawner(X/Y)" in je `TrainingArea` hebt (zie setup stap 1).
2.  **Voeg Component toe:** Selecteer het "Spawner(X/Y)" GameObject en voeg het `SpawnerScript` component toe via "Add Component" in de Inspector.
3.  **Configureer in Inspector:** Vul de publieke variabelen van het script in:
    * **Object Prefabs:** Sleep de `Obstakel` prefab vanuit je Project venster naar het `Obstacle Prefab` veld. Doe hetzelfde voor de `Target` prefab en het `Target Prefab` veld.
    * **Spawner Gebieden:** Sleep het "SpawnerX" GameObject vanuit de Hierarchy naar het `Spawner X Area` veld. Doe hetzelfde voor "SpawnerZ" en `Spawner Z Area`. *Zorg ervoor dat deze "SpawnerX" en "SpawnerZ" GameObjects een Collider component hebben!*
    * **Spawn Timing (Seconden):** Stel de `Min Spawn Time` en `Max Spawn Time` in (bijv. 3 en 5). Dit bepaalt de wachttijd in seconden tussen spawns.
    * **Obstakel Snelheid:** Stel de `Min Speed` en `Max Speed` in (bijv. 2 en 5). Dit bepaalt de snelheid van de gespawnde objecten.
4.  **Test de Setup:**
    * Start de scene (druk op Play).
    * Observeer of obstakels en targets nu correct spawnen vanuit de zijkanten (`SpawnerX` en `SpawnerZ` gebieden).
    * Controleer of ze bewegen met een snelheid binnen het ingestelde bereik.
    * Kijk in de Console (Window -> General -> Console) of er foutmeldingen van `SpawnerScript` verschijnen die wijzen op een configuratiefout (bijv. missende prefabs of colliders op de spawn gebieden).

Na deze stappen zou je een werkend spawn-systeem moeten hebben dat de basis vormt voor de uitdaging van de agent.



### 4. De Agent Aanmaken (`JumperAgent.cs`)

Creëer en configureer de agent die gaat leren. Dit is het centrale element dat we gaan trainen.

1.  **Maak het Agent GameObject:**
    * Creëer een 3D Object dat de agent voorstelt (bijv. een Capsule of Cube) in de scene, binnen de `TrainingArea`.
    * Plaats het op de positie van het "StartArea" GameObject dat je eerder hebt gemaakt.
    * Noem het GameObject "JumperAgent".

2.  **Voeg Basis Componenten toe:**
    * Selecteer de "JumperAgent".
    * Voeg een `Rigidbody` component toe via "Add Component". Hiermee kan de agent reageren op fysica (zoals zwaartekracht en krachten). Stel `Mass`, `Drag`, `Angular Drag` in naar wens. Belangrijk: **Vries de rotatie assen (Freeze Rotation X, Y, Z)** in de Rigidbody constraints om te voorkomen dat de agent omvalt tijdens het bewegen of springen.
    * Voeg een `Collider` component toe die past bij de vorm van je agent (bijv. `Capsule Collider` of `Box Collider`). Pas de grootte en positie van de collider aan zodat deze goed om de agent heen past.

3.  **Voeg ML-Agents Componenten toe:**
    * Selecteer nog steeds de "JumperAgent".
    * Voeg het `Decision Requester` component toe. Dit component zorgt ervoor dat de agent periodiek (elke `Decision Period` stappen) een nieuwe beslissing aanvraagt bij het brein. Stel `Decision Period` in op een waarde zoals `5` of `10`. Een lagere waarde betekent vaker beslissen (kan trager zijn), een hogere waarde betekent minder vaak beslissen.
    * Voeg het `Behavior Parameters` component toe. Dit is de kernkoppeling met ML-Agents. Configureer het als volgt (zie ook screenshot):
        * `Behavior Name`: Geef een duidelijke naam, bijvoorbeeld `JumperAgent`. Deze naam moet **exact overeenkomen** met de naam die je later in het `config.yaml` bestand gebruikt voor de training.
        * `Vector Observation` -> `Space Size`: Stel dit in op **18**. Dit getal moet overeenkomen met het totale aantal observaties dat je verzamelt in de `CollectObservations` methode van je `JumperAgent.cs` script.
        * `Actions`: Definieer hier de acties die de agent kan uitvoeren:
            * Type: `Continuous Actions`: Stel het aantal in op **2** (één voor horizontale beweging (X), één voor verticale beweging (Z)).
            * Type: `Discrete Actions`: Stel het aantal branches in op **1**.
                * `Branch 0 Size`: Stel de grootte van deze branch in op **2** (actie 0 betekent 'niet springen', actie 1 betekent 'springen').

    

4.  **Maak en Voeg `JumperAgent.cs` toe:**
    * Creëer een nieuw C# MonoBehaviour script in je project (bijv. in de `Scripts` map) en noem het `JumperAgent`.
    * Voeg dit `JumperAgent.cs` component toe aan het "JumperAgent" GameObject in de Inspector.
    * **Belangrijk:** Open het script en zorg dat het erft van `Agent` (uit `Unity.MLAgents`) in plaats van `MonoBehaviour`. Plak de volgende code in het script:

        ```csharp
        using System.Collections.Generic;
        using UnityEngine;
        using Unity.MLAgents;            
        using Unity.MLAgents.Sensors;
        using Unity.MLAgents.Actuators;

        public class JumperAgent : Agent 
        {
            // === INSPECTOR VARIABELEN ===

            [Header("Agent Componenten")]
            public LayerMask groundLayer;               // Welke layer(s) is de grond?
            public float jumpForce = 10.0f;             // Kracht van de sprong
            public float moveForce = 1.0f;              // Kracht voor beweging

            [Header("Observatie Instellingen")]
            public float fallThreshold = -5.0f;         // Y-positie waaronder de agent 'valt'

            [Header("Reward Settings")] // Beloningen en straffen
            public float aliveReward = 0.001f;          // Kleine beloning per stap om actief te blijven
            public float jumpReward = 0.002f;           // Kleine beloning voor springen
            public float distanceToTargetRewardScale = 0.1f; // Schaal voor beloning nabijheid target
            public string targetTag = "Target";         // Tag voor de target objecten

            [Header("Ground Check Settings")] 
            public float groundCheckSphereRadius = 0.3f; // Radius van de sphere check
            public float groundCheckDistance = 0.4f;     // Afstand van de sphere check
            public float groundCheckVerticalOffset = 0.1f; // Start offset van de sphere check

            // ===  PRIVATE VARIABELEN ===
            private Rigidbody rBody;                    // De Rigidbody component
            private bool isGrounded;                    // Staat de agent op de grond?
            private Vector3 startPosition;              // Startpositie van de agent

            // === UNITY METHODEN ===

            // Initialisatie bij de start van de agent
            public override void Initialize()
            {
                rBody = GetComponent<Rigidbody>();
                startPosition = transform.localPosition;
                // Voorkom dat de agent omvalt
                rBody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezeRotationY;
            }

            // Wordt elke physics-stap uitgevoerd
            void FixedUpdate()
            {
                isGrounded = CheckIfGrounded(); // Check grondstatus
                CheckIfFallen();                // Check of gevallen

                // Beloning om actief te blijven
                AddReward(aliveReward / (MaxStep > 0 ? MaxStep : 1000f));

                // Beloning voor dichtbij het target zijn
                RewardProximityToTarget();
            }

            // === ML-AGENTS METHODEN ===

            // Reset de agent aan het begin van een nieuwe episode
            public override void OnEpisodeBegin()
            {
                transform.localPosition = startPosition; // Terug naar startpositie
                rBody.linearVelocity = Vector3.zero;     // Reset snelheid
                rBody.angularVelocity = Vector3.zero;    // Reset rotatiesnelheid
            }

            // Verzamelt de observaties (input) voor het neurale netwerk
            public override void CollectObservations(VectorSensor sensor)
            {
                // Voeg basis observaties toe die de agent nodig heeft voor beslissingen
                sensor.AddObservation(isGrounded);                  // Weet of hij kan springen/bewegen
                sensor.AddObservation(transform.localPosition.y);   // Weet zijn hoogte
                sensor.AddObservation(rBody.linearVelocity.y);      // Weet of hij stijgt/daalt

                // BELANGRIJK: RayPerceptionSensor3D voegt ook observaties toe.
                // Totaal aantal hier (3) overeenkomen met 'Space Size' in Behavior Parameters in Unity.
            }

            // Verwerkt de acties die het netwerk heeft gekozen
            public override void OnActionReceived(ActionBuffers actions)
            {
                // Continue acties (beweging)
                float moveX = actions.ContinuousActions[0];
                float moveZ = actions.ContinuousActions[1];

                // Bewegen alleen als op de grond
                if (isGrounded)
                {
                    Vector3 moveSignal = new Vector3(moveX, 0, moveZ);
                    rBody.AddForce(moveSignal * moveForce);
                }

                // Discrete actie (springen)
                int jumpAction = actions.DiscreteActions[0];
                if (jumpAction == 1 && isGrounded)
                {
                    rBody.AddForce(Vector3.up * jumpForce, ForceMode.Impulse); // Springkracht toepassen
                    AddReward(jumpReward); // Kleine beloning voor het proberen te springen
                }
            }

            // Detecteert botsingen
            void OnCollisionEnter(Collision collision)
            {
                // Straf en einde episode bij botsing met obstakel
                if (collision.gameObject.CompareTag("Obstakel"))
                {
                    SetReward(-1.0f);
                    EndEpisode();
                }
                // Beloning en vernietig target bij botsing met target
                else if (collision.gameObject.CompareTag(targetTag))
                {
                    AddReward(1.0f);
                    Destroy(collision.gameObject);
                }
            }

            // === HELPER METHODEN ===

            // Checkt of de agent op de grond staat via een SphereCast
            private bool CheckIfGrounded()
            {
                Vector3 sphereStart = transform.position + Vector3.up * groundCheckVerticalOffset;
                // Geeft true terug als de sphere de groundLayer raakt binnen de check afstand
                return Physics.SphereCast(
                        sphereStart,
                        groundCheckSphereRadius,
                        Vector3.down,
                        out RaycastHit _, // Details van de hit zijn niet nodig
                        groundCheckDistance,
                        groundLayer
                    );
            }

            // Checkt of de agent onder de val-drempel is gekomen
            private void CheckIfFallen()
            {
                if (transform.localPosition.y < fallThreshold)
                {
                    SetReward(-1.0f); // Straf
                    EndEpisode();
                }
            }

            // Berekent en geeft beloning voor nabijheid van het dichtstbijzijnde target
            private void RewardProximityToTarget()
            {
                GameObject closestTarget = FindClosestTarget();
                if (closestTarget != null)
                {
                    float currentDistance = Vector3.Distance(transform.position, closestTarget.transform.position);
                    // Beloning is hoger als afstand kleiner is (max 1 / (1+0) = 1)
                    float proximityReward = 1.0f / (1.0f + currentDistance);
                    AddReward(proximityReward * distanceToTargetRewardScale); // Geschaalde beloning toevoegen
                }
            }

            // Vindt het dichtstbijzijnde actieve GameObject met de targetTag
            private GameObject FindClosestTarget()
            {
                GameObject[] targets = GameObject.FindGameObjectsWithTag(targetTag);
                GameObject closest = null;
                float minDistance = float.MaxValue;

                foreach (GameObject target in targets)
                {
                    if (target == null) continue; // Veiligheidscheck

                    float distance = Vector3.Distance(transform.position, target.transform.position);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        closest = target;
                    }
                }
                return closest; // Geeft null terug als er geen targets zijn
            }

            // Handelt handmatige besturing af voor testen (Heuristic mode)
            public override void Heuristic(in ActionBuffers actionsOut)
            {
                // Koppel toetsenbord input aan continue acties (beweging)
                var continuousActionsOut = actionsOut.ContinuousActions;
                continuousActionsOut[0] = Input.GetAxisRaw("Horizontal");
                continuousActionsOut[1] = Input.GetAxisRaw("Vertical");

                // Koppel Spatiebalk aan discrete actie (springen)
                var discreteActionsOut = actionsOut.DiscreteActions;
                discreteActionsOut.Clear(); // Belangrijk!
                discreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0;
            }

        }
        ```

**5. Configureer `Behavior Parameters`:** (Nieuw apart punt)

Dit component definieert hoe de agent communiceert met het ML-Agents framework (het 'brein').

* **Voeg Component toe:** Als je dit nog niet had gedaan in stap 3, voeg het `Behavior Parameters` component toe aan de "JumperAgent".
* **Configureer in Inspector:**
    * `Behavior Name`: Geef een unieke naam, bijvoorbeeld `JumperAgent`. Deze naam moet **exact overeenkomen** met de naam die je later in het `config.yaml` bestand gebruikt voor de training.
    * `Vector Observation` -> `Space Size`: Stel dit in op het **correcte aantal observaties** dat je verzamelt in `CollectObservations`. In de code hierboven tellen we 15 floats (1+1+1 + 5 + 5 + 1 + 1). Pas dit getal aan als je de observaties in de code wijzigt.
    * `Actions`: Definieer hier de acties die de agent kan uitvoeren:
        * `Continuous Actions`: Stel het aantal in op **2** (voor X en Z beweging).
        * `Discrete Actions`: Stel het aantal branches in op **1** (voor springen).
            * `Branch 0 Size`: Stel de grootte van deze branch in op **2** (actie 0 = niet springen, actie 1 = springen).


5.  **Configureer `JumperAgent.cs` (Script) in Inspector:**
    * Selecteer de "JumperAgent" opnieuw.
    * Zoek het "Jumper Agent (Script)" component in de Inspector.
    * `Ground Layer`: Klik op het dropdown menu en selecteer de Layer die je voor de vloer hebt ingesteld (bijv. "Ground").
    * `Jump Force`: Stel de gewenste springkracht in (bijv. 10).
    * `Move Force`: Stel de gewenste bewegingskracht in (bijv. 1 of 10).
    * `Observation Distance`: Stel de afstand in waarbinnen de agent obstakels detecteert (bijv. 20).
    * `Fall Threshold`: Stel de Y-waarde in waaronder de agent als gevallen telt (bijv. -5.0).


    ![alt text](<Schermafbeelding 2025-04-26 200739.jpg>)

6.  **Test met Heuristic:**
    * Zet `Behavior Type` in `Behavior Parameters` tijdelijk op `Heuristic Only`.
    * Start de scene (Play).
    * **Bestuur de agent:** Gebruik de pijltjestoetsen (of WASD) en de spatiebalk.
    * **Controleer functionaliteit:**
        * Werkt bewegen en springen zoals verwacht? Reageert de agent op de krachten?
        * Worden botsingen met obstakels (`Tag="Obstakel"`) correct gedetecteerd? Krijg je de Debug.Log melding en reset de agent (episode eindigt)?
        * Worden botsingen met targets (`Tag="Target"`) correct gedetecteerd? Krijg je de Debug.Log melding en wordt het target vernietigd?
        * Reset de agent als hij van het platform valt (onder `Fall Threshold`)?
        * Check de Console op `Debug.Log` berichten van `CheckIfGrounded` om te zien of de gronddetectie werkt.
        * Check de Console op eventuele andere foutmeldingen.

Als de Heuristic test succesvol is, heb je een werkende agent die klaar is om getraind te worden! De volgende stap is het opzetten van het configuratiebestand voor de training en het fine-tunen van de rewards.


### 5. De Agent Trainen

Nu de omgeving en de agent zijn ingesteld en getest, kunnen we de agent trainen met reinforcement learning.

1.  **Voorbereiding:**
    * Zorg dat je de ML-Agents Python package geïnstalleerd hebt in een geschikte Python-omgeving (bijv. via Conda of venv). Zie de [officiële ML-Agents installatiegids](https://github.com/Unity-Technologies/ml-agents/blob/develop/docs/Installation.md) als je dit nog niet gedaan hebt.
    * Maak een configuratiebestand aan voor de training. Noem dit bestand `JumperAgent.yaml` en plaats het ergens in je project (of daarbuiten, zolang je het pad weet). De inhoud van dit bestand bepaalt hoe de agent leert. Zie de sectie "Configuratie (`JumperAgent.yaml`)" hieronder voor een voorbeeld.
    * Open een terminal of command prompt en navigeer naar de map boven de locatie van je `JumperAgent.yaml` bestand, of zorg dat je het volledige pad naar het bestand bij de hand hebt.

2.  **Start de Training:**
    * Activeer je Python-omgeving waarin `mlagents` geïnstalleerd is.
    * Voer het volgende commando uit in je terminal (vervang `pad/naar/JumperAgent.yaml` door het correcte pad naar jouw bestand):

        ```bash
        mlagents-learn pad/naar/JumperAgent.yaml --run-id=JumperAgent_TrainRun1
        ```

        * `mlagents-learn`: Het commando om de training te starten.
        * `pad/naar/JumperAgent.yaml`: Het configuratiebestand met de hyperparameters.
        * `--run-id=JumperAgent_TrainRun1`: Een unieke naam voor deze trainingsrun. Handig om verschillende experimenten uit elkaar te houden. Verander dit (bv. naar `..._TrainRun2`) als je opnieuw traint.

    * Wacht tot de terminal de melding geeft: `[INFO] Listening on port 5004. Start training by pressing the Play button in the Unity Editor.`
    * Ga terug naar de Unity Editor en druk op de **Play** knop.

3.  **Training Observeren:**
    * De training begint nu. Je zult zien dat de agent (of meerdere instanties, als je de scene hebt gedupliceerd) acties begint uit te voeren in de scene.
    * In de terminal zie je periodiek statistieken verschijnen over de voortgang (zoals gemiddelde beloning, episode lengte).
    * Gebruik **TensorBoard** voor gedetailleerde grafieken (zie volgende sectie).

4.  **Stoppen en Resultaten:**
    * De training stopt automatisch na het bereiken van `max_steps` (gedefinieerd in de YAML).
    * Je kunt de training ook manueel stoppen door **Ctrl+C** in te drukken in de terminal. Het getrainde model (.onnx bestand) wordt dan opgeslagen in de `results/<run-id>/` map.
    * Om het getrainde model te gebruiken, sleep je het `.onnx` bestand naar je Unity project en wijs je het toe aan het `Model` veld in de `Behavior Parameters` component van je agent. Zet `Behavior Type` dan op `Inference Only`.

### 6. Configuratie (`JumperAgent.yaml`)

Het `JumperAgent.yaml` bestand bevat de instellingen voor het trainingsalgoritme  en het neural network.

```yaml
behaviors:
  JumperAgent:                # Moet EXACT overeenkomen met 'Behavior Name' in Unity!
    trainer_type: ppo         # Het gebruikte RL algoritme (Proximal Policy Optimization)
    hyperparameters:          # Instellingen die het leerproces beïnvloeden
      batch_size: 1024        # Hoeveel ervaringen per update stap
      buffer_size: 10240      # Hoeveel ervaringen onthouden we in totaal
      learning_rate: 3.0e-4   # Hoe snel past het netwerk zich aan (0.0003)
      beta: 5.0e-3            # Sterkte van de 'entropy regularization' (helpt exploratie)
      epsilon: 0.2            # Hoeveel mag het nieuwe policy afwijken van het oude? (PPO clip)
      lambd: 0.95             # Lambda voor Generalized Advantage Estimation (GAE)
      num_epoch: 3            # Hoe vaak leren we van elke batch ervaringen?
      learning_rate_schedule: linear # Hoe verandert de learning rate over tijd (hier: neemt lineair af)

    network_settings:         # Instellingen voor het neurale netwerk zelf
      normalize: false        # Normaliseer input observaties? (Hier uit, kan helpen bij variërende input)
      hidden_units: 128       # Aantal neuronen per verborgen laag
      num_layers: 2           # Aantal verborgen lagen
      vis_encode_type: simple # Hoe worden visuele observaties (indien aanwezig) verwerkt? (Hier: simple, geen visuele input)

    reward_signals:           # Definieert hoe beloningen worden verwerkt
      extrinsic:              # De 'hoofd' beloning die we in C# code geven (AddReward/SetReward)
        gamma: 0.99           # Discount factor (hoeveel waarde hechten we aan toekomstige beloningen?)
        strength: 1.0         # Hoe zwaar telt deze beloning mee?

    max_steps: 500000         # Totaal aantal simulatiestappen voor de training stopt
    time_horizon: 64          # Hoeveel stappen per agent voor we data naar buffer sturen
    summary_freq: 10000       # Hoe vaak schrijven we statistieken weg voor TensorBoard?
```

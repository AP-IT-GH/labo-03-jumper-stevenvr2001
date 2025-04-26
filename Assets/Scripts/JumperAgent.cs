using System.Collections.Generic;
        using UnityEngine;
        using Unity.MLAgents;
        using Unity.MLAgents.Sensors;
        using Unity.MLAgents.Actuators;
        using System.Linq;

        public class JumperAgent : Agent
        {
            // === INSPECTOR VARIABELEN ===
            [Header("Agent Componenten")]
            public LayerMask groundLayer;     // Layer(s) die als grond tellen
            public float jumpForce = 10.0f;   // Kracht voor de sprong
            public float moveForce = 1.0f;    // Kracht voor horizontale beweging

            [Header("Observatie Instellingen")]
            public float observationDistance = 20f; // Hoe ver de agent zoekt naar obstakels
            public float fallThreshold = -5.0f;     // Onder deze Y-positie eindigt de episode

            // ===  PRIVATE VARIABELEN ===
            private Rigidbody rBody;
            private bool isGrounded;
            private Vector3 startPosition;     // Startpositie voor reset

            // === UNITY METHODEN ===

            public override void Initialize() // Vervangt Start() voor ML-Agents initialisatie
            {
                rBody = GetComponent<Rigidbody>();
                startPosition = transform.localPosition; // Sla startpositie op

                // Stel Rigidbody constraints hier in
                rBody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezeRotationY; // Beperk rotatie/kantelen
            }

            void FixedUpdate()
            {
                isGrounded = CheckIfGrounded();
                CheckIfFallen(); // Check of agent gevallen is
            }

            // === ML-AGENTS METHODEN ===

            public override void OnEpisodeBegin()
            {
                // Reset agent positie en snelheid
                transform.localPosition = startPosition;
                rBody.linearVelocity = Vector3.zero;
                rBody.angularVelocity = Vector3.zero;
            }
            public override void CollectObservations(VectorSensor sensor)
            {
                // --- Vind relevante obstakels ---
                List<ObstacleMovement> nearbyObstacles = FindNearbyObstacles();
                ObstacleMovement closestX = null;
                ObstacleMovement closestZ = null;
                float minXDist = observationDistance;
                float minZDist = observationDistance;

                foreach (var obs in nearbyObstacles)
                {
                    if (obs == null) continue;
                    Vector3 relativePos = obs.transform.position - transform.position;
                    if (Mathf.Abs(obs.MoveDirection.x) > 0.5f)
                    {
                        if (relativePos.x > 0 && relativePos.x < minXDist) { minXDist = relativePos.x; closestX = obs; }
                    }
                    else if (Mathf.Abs(obs.MoveDirection.z) > 0.5f)
                    {
                        if (relativePos.z > 0 && relativePos.z < minZDist) { minZDist = relativePos.z; closestZ = obs; }
                    }
                }

                // --- Voeg Observaties Toe --- 
                sensor.AddObservation(isGrounded);
                sensor.AddObservation(transform.localPosition.y);
                sensor.AddObservation(rBody.linearVelocity.y);
                AddObstacleObservations(sensor, closestX);
                AddObstacleObservations(sensor, closestZ);
            }

            // Helper functie 
            private void AddObstacleObservations(VectorSensor sensor, ObstacleMovement obstacle)
            {
                if (obstacle != null)
                {
                    sensor.AddObservation(true);
                    Vector3 relativePos = obstacle.transform.position - transform.position;
                    sensor.AddObservation(relativePos.x);
                    sensor.AddObservation(relativePos.y);
                    sensor.AddObservation(relativePos.z);
                    sensor.AddObservation(obstacle.CurrentSpeed);
                }
                else
                {
                    sensor.AddObservation(false);
                    sensor.AddObservation(0f);
                    sensor.AddObservation(0f);
                    sensor.AddObservation(0f);
                    sensor.AddObservation(0f);
                }
            }

            // Acties verwerken
            public override void OnActionReceived(ActionBuffers actions)
            {
                // Continue Acties voor beweging
                float moveX = actions.ContinuousActions[0]; // Index 0 = Horizontaal
                float moveZ = actions.ContinuousActions[1]; // Index 1 = Verticaal/Vooruit
                
                if (isGrounded)
                {
                    Vector3 moveSignal = new Vector3(moveX, 0, moveZ);
                    rBody.AddForce(moveSignal * moveForce);

                }

                // Discrete Actie voor springen
                int jumpAction = actions.DiscreteActions[0]; // Index 0 = Springen branche
                if (jumpAction == 1 && isGrounded)
                {
                    rBody.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
                }
            }

            // Collision afhandeling 
            void OnCollisionEnter(Collision collision)
            {
                if (collision.gameObject.CompareTag("Obstakel"))
                {
                    SetReward(-1.0f);
                    Debug.Log("Agent geraakt door obstakel! :( "); // Debug log
                    EndEpisode();
                }
                else if (collision.gameObject.CompareTag("Target"))
                {
                    AddReward(1.0f);
                    Debug.Log("Agent heeft een target gevangen! :) "); // Debug log
                    Destroy(collision.gameObject);
                }
            }

            // === HELPER METHODEN ===

            private bool CheckIfGrounded()
            {
                // check of de agent op y 0.5 staat en of hij op een ground layer staat
                // Debug.Log("CheckIfGrounded aanroepen!"); // Debug log
                RaycastHit hit;
                if (Physics.Raycast(transform.position, Vector3.down, out hit, 1.0f, groundLayer))
                {
                    Debug.Log("Agent is op de grond!"); // Debug log
                    return true;
                }
                else
                {
                    Debug.Log("Agent is niet op de grond!"); // Debug log
                    return false;
                }
            }

            private List<ObstacleMovement> FindNearbyObstacles() // Code ongewijzigd
            {
                ObstacleMovement[] allObstacles = FindObjectsOfType<ObstacleMovement>();
                List<ObstacleMovement> nearby = new List<ObstacleMovement>();
                foreach (var obs in allObstacles)
                {
                    if(obs == null) continue;
                    if (Vector3.Distance(transform.position, obs.transform.position) < observationDistance)
                    {
                        nearby.Add(obs);
                    }
                }
                return nearby;
            }

            // Check of de agent is gevallen 
            private void CheckIfFallen()
            {
                if (transform.localPosition.y < fallThreshold)
                {
                    SetReward(-1.0f); // Straf voor vallen
                    Debug.Log("Agent is gevallen! :("); // Debug log
                    EndEpisode();
                }
            }

            // Functie voor heuristische acties
            public override void Heuristic(in ActionBuffers actionsOut)
            {
                // Debug.Log("Heuristic aanroepen!"); // Debug log

                // Continue acties voor beweging (Indices 0 en 1) -> Naar ContinuousActions buffer
                var continuousActionsOut = actionsOut.ContinuousActions;
                continuousActionsOut[0] = Input.GetAxisRaw("Horizontal"); // Gebruik GetAxisRaw voor directe input
                continuousActionsOut[1] = Input.GetAxisRaw("Vertical");

                // Discrete actie voor springen (Index 0) -> Naar DiscreteActions buffer
                var discreteActionsOut = actionsOut.DiscreteActions;
                discreteActionsOut.Clear(); // Belangrijk voor discrete acties!
                discreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0; // 1 als Spatie ingedrukt, anders 0
            }
        }
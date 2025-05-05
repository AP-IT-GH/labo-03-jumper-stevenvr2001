using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;                // Basis ML-Agents functionaliteit
using Unity.MLAgents.Sensors;       // Voor sensoren (zoals Ray Perception)
using Unity.MLAgents.Actuators;     // Voor het ontvangen van acties

public class JumperAgent : Agent // Onze agent erft van de ML-Agents Agent klasse
{
    // === INSPECTOR VARIABELEN ===
    // Deze variabelen kan je instellen in de Unity Editor.
    [Header("Agent Componenten")]
    public LayerMask groundLayer;               // Welke physics layer is de grond?
    public float jumpForce = 10.0f;             // Hoe hard springen we?
    public float moveForce = 1.0f;              // Hoe hard bewegen we?

    [Header("Observatie Instellingen")]
    public float fallThreshold = -5.0f;         // Y-grens waaronder de agent af is.

    [Header("Reward Settings")] // Instellingen voor beloningen/straffen
    public float aliveReward = 0.001f;          // Kleine beloning per stap om actief te blijven.
    public float jumpReward = 0.002f;           // Kleine beloning voor het initiëren van een sprong.
    public float velocityRewardScale = 0.05f;   // Schaal voor beloning voor snelheid *richting* het target.
    public string targetTag = "Target";         // De tag die onze target objecten hebben.

    [Header("Ground Check Settings")] // Instellingen voor de gronddetectie check.
    public float groundCheckSphereRadius = 0.3f; // Radius van de check-sphere.
    public float groundCheckDistance = 0.4f;     // Check afstand onder de agent.
    public float groundCheckVerticalOffset = 0.1f; // Startpunt van de check t.o.v. agent pivot.

    // ===  PRIVATE VARIABELEN ===
    // Deze zijn intern voor het script.
    private Rigidbody rBody;                    // Referentie naar de Rigidbody component.
    private bool isGrounded;                    // Houdt bij of de agent grond raakt.
    private Vector3 startPosition;              // De startpositie aan het begin van een episode.

    // === UNITY METHODEN ===

    // Initialisatie code, wordt 1x uitgevoerd bij start.
    public override void Initialize()
    {
        rBody = GetComponent<Rigidbody>();          // Link de Rigidbody.
        startPosition = transform.localPosition;    // Onthoud startpositie.
        // Zet rotatie vast zodat de agent niet omvalt.
        rBody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezeRotationY;
    }

    // Wordt elke vaste physics stap uitgevoerd.
    void FixedUpdate()
    {
        isGrounded = CheckIfGrounded();     // Update grondstatus.
        CheckIfFallen();                    // Check of te laag gevallen.

        // Geef altijd een kleine beloning om passiviteit tegen te gaan.
        AddReward(aliveReward / (MaxStep > 0 ? MaxStep : 1000f));

        // Geef beloning voor bewegen richting een target.
        RewardTargetSeeking();
    }

    // === ML-AGENTS METHODEN ===

    // Wordt uitgevoerd aan het begin van elke nieuwe trainingsepisode.
    public override void OnEpisodeBegin()
    {
        transform.localPosition = startPosition;    // Reset positie.
        rBody.linearVelocity = Vector3.zero;        // Reset snelheid.
        rBody.angularVelocity = Vector3.zero;       // Reset draaisnelheid.
    }

    // Verzamelt de 'zintuigen' van de agent; wat ziet/voelt hij?
    public override void CollectObservations(VectorSensor sensor)
    {
        // Voeg hier de basisinfo toe die de agent nodig heeft.
        sensor.AddObservation(isGrounded);                  // 1. Is hij op de grond?
        sensor.AddObservation(transform.localPosition.y);   // 2. Wat is zijn hoogte?
        sensor.AddObservation(rBody.linearVelocity.y);      // 3. Gaat hij omhoog of omlaag?

        // Belangrijk: De RayPerceptionSensor voegt zijn eigen observaties toe.
        // Zorg dat 'Space Size' in Behavior Parameters = 3 + aantal van RaySensor.
    }

    // Ontvangt en verwerkt de acties die het neurale netwerk kiest.
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Haal de continue acties op (voor beweging, waarden tussen -1 en 1).
        float moveX = actions.ContinuousActions[0]; // Links/Rechts
        float moveZ = actions.ContinuousActions[1]; // Vooruit/Achteruit

        // Alleen bewegen als we op de grond staan.
        if (isGrounded)
        {
            Vector3 moveSignal = new Vector3(moveX, 0, moveZ); // Creëer bewegingsvector.
            rBody.AddForce(moveSignal * moveForce);         // Pas force toe.
        }

        // Haal de discrete actie op (voor springen, waarde 0 of 1).
        int jumpAction = actions.DiscreteActions[0];
        // Alleen springen als we op de grond staan en de actie '1' is.
        if (jumpAction == 1 && isGrounded)
        {
            rBody.AddForce(Vector3.up * jumpForce, ForceMode.Impulse); // Directe force omhoog.
            AddReward(jumpReward); // Kleine beloning voor het proberen.
        }
    }

    // Wordt uitgevoerd bij een fysieke botsing.
    void OnCollisionEnter(Collision collision)
    {
        // Is het een obstakel? -> Straf en nieuwe episode.
        if (collision.gameObject.CompareTag("Obstakel"))
        {
            SetReward(-1.0f);
            EndEpisode(); // Start opnieuw.
        }
        // Is het een target? -> Beloning en vernietig target.
        else if (collision.gameObject.CompareTag(targetTag))
        {
            AddReward(1.0f);
            Destroy(collision.gameObject); // Haal target weg.
        }
    }

    // === HELPER METHODEN ===
    // Hulpfuncties die we intern gebruiken.

    // Checkt of de agent grond onder zich heeft via een SphereCast.
    private bool CheckIfGrounded()
    {
        // Startpunt van de sphere check.
        Vector3 sphereStart = transform.position + Vector3.up * groundCheckVerticalOffset;
        // Stuur een 'bol' naar beneden; geeft true als die de grond raakt.
        return Physics.SphereCast(
                   sphereStart,                 // Waar starten we?
                   groundCheckSphereRadius,     // Hoe breed is de check?
                   Vector3.down,                // Richting? Naar beneden.
                   out RaycastHit _,            // Details van wat geraakt is (hebben we niet nodig).
                   groundCheckDistance,         // Hoe ver checken?
                   groundLayer                  // Welke layer is de grond?
            );
    }

    // Checkt of de agent te ver is gevallen.
    private void CheckIfFallen()
    {
        if (transform.localPosition.y < fallThreshold)
        {
            SetReward(-1.0f); // Straf.
            EndEpisode();     // Start opnieuw.
        }
    }

    // Geeft beloning voor bewegen richting het dichtstbijzijnde target.
    private void RewardTargetSeeking()
    {
        GameObject closestTarget = FindClosestTarget(); // Zoek target.
        if (closestTarget != null) // Alleen als er een target is:
        {
            // Richting van agent naar target.
            Vector3 directionToTarget = (closestTarget.transform.position - transform.position).normalized;
            // Component van de agent's snelheid in die richting (hoe hard gaat hij eropaf?).
            float velocityTowardsTarget = Vector3.Dot(rBody.linearVelocity, directionToTarget);

            // Als de agent daadwerkelijk die kant op beweegt:
            if (velocityTowardsTarget > 0.1f) // Kleine drempel tegen beloning bij stilstaan.
            {
                // Geef beloning die schaalt met hoe snel hij die kant op gaat.
                AddReward(velocityTowardsTarget * velocityRewardScale);
            }
        }
    }

    // Vindt het dichtstbijzijnde actieve GameObject met de targetTag.
    private GameObject FindClosestTarget()
    {
        GameObject[] targets = GameObject.FindGameObjectsWithTag(targetTag); // Vind alle targets.
        GameObject closest = null;
        float minDistance = float.MaxValue; // Begin check met oneindige afstand.

        // Check elk target.
        foreach (GameObject target in targets)
        {
            if (target == null) continue; // Voor het geval een target net vernietigd is.

            float distance = Vector3.Distance(transform.position, target.transform.position); // Afstand berekenen.
            // Als deze target dichterbij is dan het huidige dichtstbijzijnde:
            if (distance < minDistance)
            {
                minDistance = distance; // Onthoud nieuwe kleinste afstand.
                closest = target;      // Onthoud deze target.
            }
        }
        return closest; // Geef dichtstbijzijnde target terug (of null als er geen zijn).
    }

    // Regelt de handmatige besturing voor testen (Heuristic mode in Behavior Parameters).
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Koppel pijltjes/WASD aan continue acties (beweging).
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxisRaw("Horizontal"); // Links/Rechts (-1, 0, 1)
        continuousActionsOut[1] = Input.GetAxisRaw("Vertical");   // Voor/Achter (-1, 0, 1)

        // Koppel Spatiebalk aan discrete actie (springen).
        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut.Clear(); // Heel belangrijk voor discrete acties!
        discreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0; // 1 = Spatie ingedrukt, 0 = niet.
    }
}
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
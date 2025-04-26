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
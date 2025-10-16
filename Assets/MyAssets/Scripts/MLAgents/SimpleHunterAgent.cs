using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Agent ayant pour t�che de chasser les PreyAgents
/// </summary>
[
    RequireComponent(typeof(Rigidbody)),
    RequireComponent(typeof(DecisionRequester)),
]
public class SimpleHunterAgent : Agent
{
    #region inspector fields

    [Header("Agent settings")]
    [SerializeField] Rigidbody _rigidbody;

    [SerializeField] Vector3 _defaultAgentInitialPosition = Vector3.zero;
    [SerializeField] float _defaultAgentAcceleration = 5f;

    [Header("Target settings")]
    [SerializeField] Transform _target;
    [SerializeField] Vector3 _defaultTargetInitialPosition = Vector3.forward;

    [Header("GameBoard settings")]
    [SerializeField] Transform _gameBoard;

    [Header("Heuristic")]
    [SerializeField] InputActionReference _moveAction;

    [Header("Debug")]
    [SerializeField] bool _displayGUI;

    #endregion


    #region private fields

    EnvironmentParameters _envParameters;

    Vector3 _agentInitialPosition;
    float _agentAcceleration;

    Vector3 _targetInitialPosition;

#if UNITY_EDITOR
    private void OnValidate()
    {
        if (!TryGetComponent(out _rigidbody))
            _rigidbody = gameObject.AddComponent<Rigidbody>();
    }

    private void OnGUI()
    {
        if(_displayGUI)
        {
            GUILayout.BeginVertical();

            GUILayout.Label("Heuristic move input: " + _moveAction.action.ReadValue<Vector2>().ToString("N3"));
            GUILayout.Label("Linear Velocity: " + _rigidbody.linearVelocity.ToString("N5"));

            GUILayout.EndVertical();
        }
    }
#endif

    protected override void OnEnable()
    {
        base.OnEnable();

        _moveAction.action.Enable();
    }

    protected override void OnDisable()
    {
        base.OnDisable();

        _moveAction.action.Disable();
    }

    #endregion
    /// <summary>
    /// Initialize est appel�e au d�but de l'entra�nement.
    /// Utile pour se connecter � l'Academy qui servira�  r�cup�rer les param�tres d'environnement
    /// dans le fichier de configuration de l'entra�nement.
    /// </summary>
    public override void Initialize()
    {
        _envParameters = Academy.Instance.EnvironmentParameters;

        if(!TryGetComponent(out _rigidbody))
            _rigidbody = gameObject.AddComponent<Rigidbody>();
    }

    /// <summary>
    /// OnEpisodeBegin est appel�e � chaque d�but d'�pisode
    /// </summary>
    public override void OnEpisodeBegin()
    {
        // R�cup�ration des param�tres d'environnement pour l'agent et sa cible
        _agentInitialPosition.x = _envParameters.GetWithDefault("agent-initial-position-x", _defaultAgentInitialPosition.x);
        _agentInitialPosition.z = _envParameters.GetWithDefault("agent-initial-position-z", _defaultAgentInitialPosition.z);

        _agentAcceleration = _envParameters.GetWithDefault("agent-acceleration", _defaultAgentAcceleration);

        _targetInitialPosition.x = _envParameters.GetWithDefault("target-initial-position-x", _defaultTargetInitialPosition.x);
        _targetInitialPosition.z = _envParameters.GetWithDefault("target-initial-position-z", _defaultTargetInitialPosition.z);


        // Reset les v�locit�s sinon l'agent conserve son inertie de l'�pisode pr�c�dent
        _rigidbody.linearVelocity = Vector3.zero;
        _rigidbody.angularVelocity = Vector3.zero;

        // Reset de la position et de l'orientation de l'agent en utilisant les coordonn�es locales.
        // Lorsque plusieurs agents sont utilis�s simultan�ment pour entra�ner le mod�le,
        // utiliser les coordonn�es locales leur permet de s'ignorer.
        transform.localPosition = _agentInitialPosition;
        transform.localEulerAngles = Vector3.zero;

        // Reset de la position de la cible
        _target.localPosition = _targetInitialPosition;
    }

    /// <summary>
    /// CollectObservations est appel�e au d�but � la fin de chaque transition (avant et apr�s chaque action)
    /// </summary>
    /// <param name="sensor">Vecteur auquel on ajoute les diff�rentes valeurs � observer</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        // Observation de la position de la cible (3 floats)
        sensor.AddObservation(_target.localPosition);

        // Observation de la position de l'agent (3 floats)
        sensor.AddObservation(transform.localPosition);

        // Observation de la v�locit� lin�aire sur les axes x & z (2 floats)
        sensor.AddObservation(_rigidbody.linearVelocity.x);
        sensor.AddObservation(_rigidbody.linearVelocity.z);
    }

    /// <summary>
    /// OnActionReceived est appel�e � chaque frame et permet de transformer les valeurs envoy�es par le Brain en action dans l'environnement
    /// </summary>
    /// <param name="actions">Buffer contenant les actions continues et/ou discr�tes</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // On r�cup�re les valeurs des actions continues afin de les retranscrire en acc�l�ration
        float zAcceleration = actions.ContinuousActions[0] * _agentAcceleration;
        float xAcceleration = actions.ContinuousActions[1] * _agentAcceleration;

        // On d�placer l'agent en utilisant son Rigidbody et les acc�l�rations
        _rigidbody.AddForce(
            transform.position + 
            Vector3.forward * zAcceleration +
            Vector3.right * xAcceleration
        );

        // Si l'agent tombe sous le niveau du plan, on met fin � l'�pisode pour recommencer
        if(transform.position.y < 0)
            EndEpisode();
    }

    /// <summary>
    /// M�thode appel�e lorsqu'aucun Brain (interne ou externe) n'est connect� � l'agent.
    /// Permet de tester les actions de l'agent dans son environnement.
    /// </summary>
    /// <param name="actionsOut"></param>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;
        actions[0] = _moveAction.action.ReadValue<Vector2>().y;
        actions[1] = _moveAction.action.ReadValue<Vector2>().x;
    }


    private void OnCollisionEnter(Collision collision)
    {
        // Si l'agent entre en collision avec sa cible, on le r�compense et on met fin � l'�pisode
        if(collision.transform == _target)
        {
            SetReward(1f);
            EndEpisode();
        }
    }
}

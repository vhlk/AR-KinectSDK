using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Kinect = Windows.Kinect;
using UnityEngine.UI;

public class BodySourceView : MonoBehaviour 
{
    public Material BoneMaterial;
    public GameObject BodySourceManager;

    private const int LEFT_ARM = 0;
    private const int RIGHT_ARM = 1;

    public Text angleText;
    public Text titleText;
    public Text alertText;
    public Text armBodyAngleText;

    private int minAngle = 45;
    private int maxAngle = 90;

    public InputField minAngleField;
    public InputField maxAngleField;
    public InputField armBodyAngleField;

    private LineRenderer arrow;
    private float percentHead = 0.4f;

    public Dropdown armDropdown;

    private int armSide = 0;
    private int armBodyAngle = -1;

    public Toggle alignArmWithBodyToggle;

    private bool alignArmWithBody;

    private Plane bodyPlane;

    private float closeRate = 0.3f; // representa a porcentagem em relação à diferença entre o ângulo máximo e o mínimo que será considerado próximo
    private float armBodyPrecision = 5;
    
    private Dictionary<ulong, GameObject> _Bodies = new Dictionary<ulong, GameObject>();
    private BodySourceManager _BodyManager;
    
    private Dictionary<Kinect.JointType, Kinect.JointType> _BoneMap = new Dictionary<Kinect.JointType, Kinect.JointType>()
    {
        { Kinect.JointType.FootLeft, Kinect.JointType.AnkleLeft },
        { Kinect.JointType.AnkleLeft, Kinect.JointType.KneeLeft },
        { Kinect.JointType.KneeLeft, Kinect.JointType.HipLeft },
        { Kinect.JointType.HipLeft, Kinect.JointType.SpineBase },
        
        { Kinect.JointType.FootRight, Kinect.JointType.AnkleRight },
        { Kinect.JointType.AnkleRight, Kinect.JointType.KneeRight },
        { Kinect.JointType.KneeRight, Kinect.JointType.HipRight },
        { Kinect.JointType.HipRight, Kinect.JointType.SpineBase },
        
        { Kinect.JointType.HandTipLeft, Kinect.JointType.HandLeft },
        { Kinect.JointType.ThumbLeft, Kinect.JointType.HandLeft },
        { Kinect.JointType.HandLeft, Kinect.JointType.WristLeft },
        { Kinect.JointType.WristLeft, Kinect.JointType.ElbowLeft },
        { Kinect.JointType.ElbowLeft, Kinect.JointType.ShoulderLeft },
        { Kinect.JointType.ShoulderLeft, Kinect.JointType.SpineShoulder },
        
        { Kinect.JointType.HandTipRight, Kinect.JointType.HandRight },
        { Kinect.JointType.ThumbRight, Kinect.JointType.HandRight },
        { Kinect.JointType.HandRight, Kinect.JointType.WristRight },
        { Kinect.JointType.WristRight, Kinect.JointType.ElbowRight },
        { Kinect.JointType.ElbowRight, Kinect.JointType.ShoulderRight },
        { Kinect.JointType.ShoulderRight, Kinect.JointType.SpineShoulder },
        
        { Kinect.JointType.SpineBase, Kinect.JointType.SpineMid },
        { Kinect.JointType.SpineMid, Kinect.JointType.SpineShoulder },
        { Kinect.JointType.SpineShoulder, Kinect.JointType.Neck },
        { Kinect.JointType.Neck, Kinect.JointType.Head },
    };
    
    void Update () 
    {
        if (BodySourceManager == null)
        {
            return;
        }
        
        _BodyManager = BodySourceManager.GetComponent<BodySourceManager>();
        if (_BodyManager == null)
        {
            return;
        }
        
        Kinect.Body[] data = _BodyManager.GetData();
        if (data == null)
        {
            return;
        }
        
        List<ulong> trackedIds = new List<ulong>();
        foreach(var body in data)
        {
            if (body == null)
            {
                continue;
              }
                
            if(body.IsTracked)
            {
                trackedIds.Add (body.TrackingId);
            }
        }
        
        List<ulong> knownIds = new List<ulong>(_Bodies.Keys);
        
        // First delete untracked bodies
        foreach(ulong trackingId in knownIds)
        {
            if(!trackedIds.Contains(trackingId))
            {
                Destroy(_Bodies[trackingId]);
                _Bodies.Remove(trackingId);
            }
        }

        foreach(var body in data)
        {
            if (body == null)
            {
                continue;
            }

            if (body.IsTracked)
            {
                if (!_Bodies.ContainsKey(body.TrackingId))
                {
                    _Bodies[body.TrackingId] = CreateBodyObject(body.TrackingId);
                }

                RefreshBodyObject(body, _Bodies[body.TrackingId]);

                // calculamos o ângulo que é formado na região do cotovelo (para o braço esquerdo)
                float angle = CalculateArmAngle(body);
                if (angle == -1)
                {
                    angleText.text = "Por favor ajuste o braço para aparecer na tela";
                    continue;
                }
                if (angle == -2)
                {
                    angleText.text = "Erro: seleção de braço inválida";
                    continue;
                }
                angleText.text = angle.ToString("F") + "°";

                // mudar cor do braço dependendo da angulação
                ChangeArmColorOnAngle(angle, body);

                UpdateArrow(body, angle);

                // caso o usuário tenha selecionado a opção de alinhar o braço com o corpo
                if (alignArmWithBody)
                {
                    // verificamos se foi possível criar o plano que representa o corpo
                    bool isBodyVisible = UpdBodyPlane(body);
                    if (!isBodyVisible)
                    {
                        alertText.text = "Corpo não detectado";
                        alertText.enabled = true;
                        continue;
                    }
                    if (!isArmAligned(body))
                    {
                        alertText.text = "Alinhe o braço!";
                        alertText.enabled = true;
                        continue;
                    }
                }

                if (armBodyAngle != -1)
                {
                    float currentArmBodyAngle = CalculateArmBodyAngle(body);
                    if (currentArmBodyAngle == -1)
                    {
                        alertText.text = "Cintura e/ou braços não detectados";
                        alertText.enabled = true;
                        continue;
                    }
                    else
                    {
                        if (armBodyAngle + armBodyPrecision < currentArmBodyAngle ||
                            currentArmBodyAngle < armBodyAngle - armBodyPrecision)
                        {
                            alertText.text = "Ajuste a abertura do braço para um ângulo de " + armBodyAngle + "°";
                            alertText.enabled = true;
                        }
                        else
                        {
                            alertText.enabled = false;
                        }
                        armBodyAngleText.text = "Ângulo atual: " + currentArmBodyAngle + "°";
                        armBodyAngleText.enabled = true;
                    }
                }
                if (armBodyAngle == -1) armBodyAngleText.enabled = false;

                // desativamos o alerta caso esteja tudo certo
                if (armBodyAngle == -1) alertText.enabled = false;
            }
        }
        if (arrow != null && trackedIds.Count == 0) arrow.enabled = false;
        if (armBodyAngleText.enabled && trackedIds.Count == 0) armBodyAngleText.enabled = false;
    }

    private float CalculateArmAngle(Kinect.Body body)
    {
        /* Entrada: Kinect.Body body
         * Saída: float ângulo formado na região do cotovelo, caso o braço esquerdo esteja visível. Se não, retorna -1
         */

        if (armSide == LEFT_ARM)
        {
            // verificamos se há visibilidade do pulso, cotovelo e ombro
            if (areLeftArmPointsVisibles(body))
            {
                // pegamos a posição de cada um e calculamos o ângulo Pulso-Cotovelo-Ombro
                Vector3 posWristLeft = GetVector3FromJoint(body.Joints[Kinect.JointType.WristLeft]);
                Vector3 posElbowLeft = GetVector3FromJoint(body.Joints[Kinect.JointType.ElbowLeft]);
                Vector3 posShoulderLeft = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderLeft]);
                float angle = Vector3.Angle(posWristLeft - posElbowLeft, posShoulderLeft - posElbowLeft);
                return angle;
            }
            return -1;
        }
        else if (armSide == RIGHT_ARM)
        {
            // verificamos se há visibilidade do pulso, cotovelo e ombro
            if (areRightArmPointsVisibles(body))
            {
                // pegamos a posição de cada um e calculamos o ângulo Pulso-Cotovelo-Ombro
                Vector3 posWristRight = GetVector3FromJoint(body.Joints[Kinect.JointType.WristRight]);
                Vector3 posElbowRight = GetVector3FromJoint(body.Joints[Kinect.JointType.ElbowRight]);
                Vector3 posShoulderRight = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderRight]);
                float angle = Vector3.Angle(posWristRight - posElbowRight, posShoulderRight - posElbowRight);
                return angle;
            }
            return -1;
        }
        return -2;
    }

    private void ChangeArmColorOnAngle(float angle, Kinect.Body body)
    {
        /* Entrada: float angle, representando a angulação do braço esquerdo;
         *          Kinect.Body body
         */


        // pegamos objeto que representa a pessoa
        GameObject bodyObject = _Bodies[body.TrackingId];

        if (armSide == LEFT_ARM)
        {
            // verificamos se há visibilidade da pulso, cotovelo e ombro
            if (areLeftArmPointsVisibles(body))
            {
                // pegar o linerenderer do ombro para o cotovelo
                Transform elbowObj = bodyObject.transform.Find(Kinect.JointType.ElbowLeft.ToString());
                LineRenderer lrShoulderElbow = elbowObj.GetComponent<LineRenderer>();

                // pegar o linerenderer do cotovelo até o pulso
                Transform wristObj = bodyObject.transform.Find(Kinect.JointType.WristLeft.ToString());
                LineRenderer lrElbowWrist = wristObj.GetComponent<LineRenderer>();

                if (angle >= minAngle && angle <= maxAngle)
                {
                    // vamos colorir o braço de verde, representando que atingiu o objetivo
                    lrShoulderElbow.SetColors(Color.green, Color.green);
                    lrElbowWrist.SetColors(Color.green, Color.green);
                }
                else if (angle <= maxAngle + closeRate * (maxAngle - minAngle) && 
                    angle > minAngle)
                {
                    // vamos nos colorir o braço de amarelo, representando estar perto
                    lrShoulderElbow.SetColors(Color.yellow, Color.yellow);
                    lrElbowWrist.SetColors(Color.yellow, Color.yellow);
                }
                else
                {
                    // vamos nos colorir o braço de vermelho, representando que ainda não conseguiu concluir o exercício
                    lrShoulderElbow.SetColors(Color.red, Color.red);
                    lrElbowWrist.SetColors(Color.red, Color.red);
                }
            }
        }
        else if (armSide == RIGHT_ARM)
        {
            // verificamos se há visibilidade da pulso, cotovelo e ombro
            if (areRightArmPointsVisibles(body))
            {
                // pegar o linerenderer do ombro para o cotovelo
                Transform elbowObj = bodyObject.transform.Find(Kinect.JointType.ElbowRight.ToString());
                LineRenderer lrShoulderElbow = elbowObj.GetComponent<LineRenderer>();

                // pegar o linerenderer do cotovelo até o pulso
                Transform wristObj = bodyObject.transform.Find(Kinect.JointType.WristRight.ToString());
                LineRenderer lrElbowWrist = wristObj.GetComponent<LineRenderer>();

                if (angle >= minAngle && angle <= maxAngle)
                {
                    // vamos colorir o braço de verde, representando que atingiu o objetivo
                    lrShoulderElbow.SetColors(Color.green, Color.green);
                    lrElbowWrist.SetColors(Color.green, Color.green);
                }
                else if (angle <= maxAngle + closeRate * (maxAngle - minAngle) &&
                    angle > minAngle)
                {
                    // vamos nos colorir o braço de amarelo, representando estar perto
                    lrShoulderElbow.SetColors(Color.yellow, Color.yellow);
                    lrElbowWrist.SetColors(Color.yellow, Color.yellow);
                }
                else
                {
                    // vamos nos colorir o braço de vermelho, representando que ainda não conseguiu concluir o exercício
                    lrShoulderElbow.SetColors(Color.red, Color.red);
                    lrElbowWrist.SetColors(Color.red, Color.red);
                }
            }
        }
    }

    private void UpdateArrow(Kinect.Body body, float angle)
    {
        /* Entrada: Kinect.Body body, representando o corpo atual
         * atualiza a curva para apontar do pulso ao ombro, caso ainda não tenha atingido o objetivo */

        if (arrow == null) arrow = GetComponent<LineRenderer>();

        // se o objetivo foi concluído ou o ângulo for inválido, desabilitamos a seta
        if (angle >= minAngle && angle <= maxAngle || angle == -1)
        {
            arrow.enabled = false;
            return;
        }

        // caso algum ponto não esteja visível, não msotramos a seta
        if (armSide == LEFT_ARM && !areLeftArmPointsVisibles(body))
        {
            arrow.enabled = false;
            return;
        }
        else if (armSide == RIGHT_ARM && !areRightArmPointsVisibles(body))
        {
            arrow.enabled = false;
            return;
        }
        else if (armSide != LEFT_ARM && armSide != RIGHT_ARM)
        {
            arrow.enabled = false;
            return;
        }

        // habilitamos a seta
        arrow.enabled = true;

        // pontos iniciais e finais da seta
        Vector3 arrowOrigin;
        Vector3 arrowEnd;

        // caso tenha passado, precisamos apontar para o lado contrário (sentido da mão)
        bool inversedArrow = false;
        if (angle < minAngle) inversedArrow = true;

        if (armSide == LEFT_ARM)
        {
            if (inversedArrow)
            {
                arrowEnd = GetVector3FromJoint(body.Joints[Kinect.JointType.WristLeft]);
                arrowOrigin = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderLeft]);
            }
            else
            {
                arrowOrigin = GetVector3FromJoint(body.Joints[Kinect.JointType.WristLeft]);
                arrowEnd = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderLeft]);
            }
        }
        else
        {
            if (inversedArrow)
            {
                arrowEnd = GetVector3FromJoint(body.Joints[Kinect.JointType.WristRight]);
                arrowOrigin = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderRight]);
            }
            else
            {
                arrowOrigin = GetVector3FromJoint(body.Joints[Kinect.JointType.WristRight]);
                arrowEnd = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderRight]);
            }
        }

        // adaptar tamanho da cabeça
        float adaptiveSize = (float)(percentHead / Vector3.Distance(arrowOrigin, arrowEnd));

        // criar desenho da seta
        arrow.widthCurve = new AnimationCurve(
            new Keyframe(0, 0.4f),
            new Keyframe(0.999f - adaptiveSize, 0.4f), // pescoço da flecha
            new Keyframe(1 - adaptiveSize, 1f), // largura max da cabeça da flecha
            new Keyframe(1, 0f) // ponta da flecha
            );

        arrow.SetPositions(new Vector3[]
        {
            arrowOrigin,
            Vector3.Lerp(arrowOrigin, arrowEnd, 0.999f - adaptiveSize),
            Vector3.Lerp(arrowOrigin, arrowEnd, 1 - adaptiveSize),
            arrowEnd
        });
    }

    public void ChangeArmSide()
    {
        /* muda o lado do braço de acordo com a seleção no dropdown */
        armSide = armDropdown.value;
    }

    private bool areLeftArmPointsVisibles(Kinect.Body body)
    {
        return body.Joints[Kinect.JointType.WristLeft].TrackingState == Kinect.TrackingState.Tracked &&
               body.Joints[Kinect.JointType.ElbowLeft].TrackingState == Kinect.TrackingState.Tracked &&
               body.Joints[Kinect.JointType.ShoulderLeft].TrackingState == Kinect.TrackingState.Tracked;
    }

    private bool areRightArmPointsVisibles(Kinect.Body body)
    {
        return body.Joints[Kinect.JointType.WristRight].TrackingState == Kinect.TrackingState.Tracked &&
               body.Joints[Kinect.JointType.ElbowRight].TrackingState == Kinect.TrackingState.Tracked &&
               body.Joints[Kinect.JointType.ShoulderRight].TrackingState == Kinect.TrackingState.Tracked;
    }

    public void ChangeMinAngle()
    {
        /* muda o ângulo mínimo, de acordo com o selecionado pelo usuário. Valor mínimo: 0, valor máximo: 90 
         * obs: o ângulo mínimo não pode ser maior que o ângulo máximo
         */

        // evitar caso de input vazio
        if (minAngleField.text == null || minAngleField.text.Equals(""))
            return;

        // verificamos a condição descrita
        int angle = int.Parse(minAngleField.text);

        // verificamos a condição descrita
        if (angle < 0 || angle > 90 || angle > maxAngle)
            return;

        // atualizamos o ângulo
        minAngle = angle;
        ChangeTitleTextAngle();
    }
    public void ChangeMaxAngle()
    {
        /* muda o ângulo máximo, de acordo com o selecionado pelo usuário. Valor mínimo: 0, valor máximo: 180 
         * obs: o ângulo máximo não pode ser menor que o ângulo mínimo
         */

        // evitar caso de input vazio
        if (maxAngleField.text == null || maxAngleField.text.Equals(""))
            return;

        int angle = int.Parse(maxAngleField.text);

        // verificamos a condição descrita
        if (angle < 0 || angle > 180 || angle < minAngle)
            return;

        // atualizamos o ângulo
        maxAngle = angle;
        ChangeTitleTextAngle();
    }

    private void ChangeTitleTextAngle()
    {
        /* atualizar os ângulos no título */

        titleText.text = "Exercício de musculatura\n" + "(Neste teste tentamos acostumar o braço a " +
            "dobrar a partir do ângulo de " + maxAngle + "° até " + minAngle + "°)";
    }

    public void ChangeArmBodyAngle()
    {
        /* muda o ângulo do braço em relação ao corpo de acordo com o digitado pelo usuário
         * Valor mínimo: 0
         * Valor máximo: 180
         */

        // verificar se o input é vazio
        if (armBodyAngleField.text == null || armBodyAngleField.text.Equals(""))
        {
            armBodyAngle = -1;
            armBodyAngleText.enabled = false;
            return;
        }

        int angle = int.Parse(armBodyAngleField.text);

        // verificamos a condição
        if (angle < 0 || angle > 180)
            return;

        armBodyAngle = angle;
    }

    private bool UpdBodyPlane(Kinect.Body body)
    {
        /* Entrada: Kinect.Body body
         * Saída: booleano representando se foi popssível localizar o plano ou não
         * aqui calculamos a direção do vetor (não importa o sentido) e verificamos se o vetor do braço está alinhado */

        if (areArmsShouldersSpineVisible(body))
        {
            Vector3 vecSpineBase = GetVector3FromJoint(body.Joints[Kinect.JointType.SpineBase]);
            Vector3 vecLeftShoulder = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderLeft]);
            Vector3 vecRightShoulder = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderRight]);

            bodyPlane = new Plane(vecSpineBase, vecLeftShoulder, vecRightShoulder);
            return true;
        }
        return false;
    }

    public void ChangeAlignWithBody()
    {
        /* recebe do usuário, através do toggle, se deve alinhar o braço com o corpo */

        alignArmWithBody = alignArmWithBodyToggle.isOn;
    }

    private bool areArmsShouldersSpineVisible(Kinect.Body body)
    {
        /* já temos uma função para calcular a visibilidade de um dos braços, 
         * precisamos verificar se o outro ombro e a parte de baixo da coluna também está visível */

        if (armSide == LEFT_ARM)
        {
            return body.Joints[Kinect.JointType.ElbowRight].TrackingState == Kinect.TrackingState.Tracked &&
                   body.Joints[Kinect.JointType.SpineBase].TrackingState == Kinect.TrackingState.Tracked &&
                   areLeftArmPointsVisibles(body);
        }
        else
        {
            return body.Joints[Kinect.JointType.ElbowLeft].TrackingState == Kinect.TrackingState.Tracked &&
                body.Joints[Kinect.JointType.SpineBase].TrackingState == Kinect.TrackingState.Tracked &&
                areRightArmPointsVisibles(body);
        }
    }

    private bool isArmAligned(Kinect.Body body)
    {
        /* Entrada: Kinect.Body bode,
         * Saída: booleano representando se o braço está alinhado com o corpo*/

        float distPointPlane;

       if (armSide == LEFT_ARM)
        {
            Vector3 posLeftElbow = GetVector3FromJoint(body.Joints[Kinect.JointType.ElbowLeft]);

            // pegamos a distância do ponto ao plano
            distPointPlane = Mathf.Abs(bodyPlane.GetDistanceToPoint(posLeftElbow));

         }
       else
        {
            Vector3 posRightElbow = GetVector3FromJoint(body.Joints[Kinect.JointType.ElbowRight]);

            // pegamos a distância do ponto ao plano
            distPointPlane = Mathf.Abs(bodyPlane.GetDistanceToPoint(posRightElbow));

        }

        if (distPointPlane <= 0.5)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private float CalculateArmBodyAngle(Kinect.Body body)
    {
        /* Entrada: Kinect.Body body
         * Saída: float representando a angulação do braço em relação ao corpo
         */

        if (armSide == LEFT_ARM)
        {
            // primeiro verificamos se os pontos estão visíveis
            if (areHipsVisible(body) && areLeftArmPointsVisibles(body)) {
                Vector3 leftHip = GetVector3FromJoint(body.Joints[Kinect.JointType.HipLeft]);
                Vector3 rightHip = GetVector3FromJoint(body.Joints[Kinect.JointType.HipRight]);
                Vector3 leftShoulder = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderLeft]);
                Vector3 leftElbow = GetVector3FromJoint(body.Joints[Kinect.JointType.ElbowLeft]);

                Vector3 vecHipHip = leftHip - rightHip;
                Vector3 vecHipShoulder = leftShoulder - leftHip;

                // calculamos a projeção do vetor Hip->Shoulder sobre o vetor Hip->Hip
                Vector3 proj = Vector3.Project(vecHipShoulder, vecHipHip);

                // calculamos um ponto que se aproxima do local do corpo onde o braço encosta
                Vector3 shoulderOnHipHeight = leftHip + proj;

                Vector3 vecBody = shoulderOnHipHeight - leftShoulder;

                Vector3 vecShoulderElbow = leftElbow - leftShoulder;

                float angle = Vector3.Angle(vecBody, vecShoulderElbow);

                return angle;
            }
        }
        else
        {
            // primeiro verificamos se os pontos estão visíveis
            if (areHipsVisible(body) && areRightArmPointsVisibles(body))
            {
                Vector3 leftHip = GetVector3FromJoint(body.Joints[Kinect.JointType.HipLeft]);
                Vector3 rightHip = GetVector3FromJoint(body.Joints[Kinect.JointType.HipRight]);
                Vector3 rightShoulder = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderRight]);
                Vector3 rightElbow = GetVector3FromJoint(body.Joints[Kinect.JointType.ElbowRight]);

                Vector3 vecHipHip = rightHip - leftHip;
                Vector3 vecHipShoulder = rightShoulder - rightHip;

                // calculamos a projeção do vetor Hip->Shoulder sobre o vetor Hip->Hip
                Vector3 proj = Vector3.Project(vecHipShoulder, vecHipHip);

                Vector3 shoulderOnHipHeight = rightHip + proj;

                // calculamos um ponto que se aproxima do local do corpo onde o braço encosta
                Vector3 vecBody = shoulderOnHipHeight - rightShoulder;

                Vector3 vecShoulderElbow = rightElbow - rightShoulder;

                float angle = Vector3.Angle(vecBody, vecShoulderElbow);

                return angle;
            }
        }
        return -1;

    }

    private bool areHipsVisible(Kinect.Body body)
    {
        return body.Joints[Kinect.JointType.HipLeft].TrackingState == Kinect.TrackingState.Tracked &&
            body.Joints[Kinect.JointType.HipRight].TrackingState == Kinect.TrackingState.Tracked;
    }

    private GameObject CreateBodyObject(ulong id)
    {
        GameObject body = new GameObject("Body:" + id);
        
        for (Kinect.JointType jt = Kinect.JointType.SpineBase; jt <= Kinect.JointType.ThumbRight; jt++)
        {
            GameObject jointObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            
            LineRenderer lr = jointObj.AddComponent<LineRenderer>();
            lr.SetVertexCount(2);
            lr.material = BoneMaterial;
            lr.SetWidth(0.05f, 0.05f);
            
            jointObj.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
            jointObj.name = jt.ToString();
            jointObj.transform.parent = body.transform;
        }
        
        return body;
    }
    
    private void RefreshBodyObject(Kinect.Body body, GameObject bodyObject)
    {
        for (Kinect.JointType jt = Kinect.JointType.SpineBase; jt <= Kinect.JointType.ThumbRight; jt++)
        {
            Kinect.Joint sourceJoint = body.Joints[jt];
            Kinect.Joint? targetJoint = null;
            
            if(_BoneMap.ContainsKey(jt))
            {
                targetJoint = body.Joints[_BoneMap[jt]];
            }
            
            Transform jointObj = bodyObject.transform.Find(jt.ToString());
            jointObj.localPosition = GetVector3FromJoint(sourceJoint);
            
            LineRenderer lr = jointObj.GetComponent<LineRenderer>();
            if(targetJoint.HasValue)
            {
                lr.SetPosition(0, jointObj.localPosition);
                lr.SetPosition(1, GetVector3FromJoint(targetJoint.Value));
                lr.SetColors(GetColorForState (sourceJoint.TrackingState), GetColorForState(targetJoint.Value.TrackingState));
            }
            else
            {
                lr.enabled = false;
            }
        }
    }
    
    private static Color GetColorForState(Kinect.TrackingState state)
    {
        switch (state)
        {
        case Kinect.TrackingState.Tracked:
            return Color.green;

        case Kinect.TrackingState.Inferred:
            return Color.red;

        default:
            return Color.black;
        }
    }
    
    private static Vector3 GetVector3FromJoint(Kinect.Joint joint)
    {
        return new Vector3(joint.Position.X * 10, joint.Position.Y * 10, joint.Position.Z * 10);
    }
}

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

    private int minAngle = 45;
    private int maxAngle = 90;

    public InputField minAngleField;
    public InputField maxAngleField;

    private LineRenderer arrow;
    private float percentHead = 0.4f;

    public Dropdown armDropdown;

    private int armSide = 0;
    
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
                    return;
                }
                if (angle == -2)
                {
                    angleText.text = "Erro: seleção de braço inválida";
                    return;
                }
                angleText.text = angle.ToString() + "°";

                // mudar cor do braço dependendo da angulação
                ChangeArmColorOnAngle(angle, body);

                UpdateArrow(body, angle);
            }
        }
        if (arrow != null && trackedIds.Count == 0) arrow.enabled = false;
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
                else if (angle <= maxAngle+30 && angle >= minAngle-30)
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
                else if (angle <= maxAngle + 30 && angle >= minAngle - 30)
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

        if (armSide == LEFT_ARM)
        {
            arrowOrigin = GetVector3FromJoint(body.Joints[Kinect.JointType.WristLeft]);
            arrowEnd = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderLeft]);
        }
        else
        {
            arrowOrigin = GetVector3FromJoint(body.Joints[Kinect.JointType.WristRight]);
            arrowEnd = GetVector3FromJoint(body.Joints[Kinect.JointType.ShoulderRight]);
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

        titleText.text = "Exercício de musculatura\n" + "Neste teste tentamos acostumar o braço a " +
            "dobrar a partir do ângulo de " + maxAngle + "° até " + minAngle + "°)";
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

using System;
using System.Collections;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using TMPro; // UI 텍스트 표시를 위해 추가

// [주의] 이 스크립트를 유니티 프로젝트의 Assets 폴더에 넣고, 아무 GameObject에 컴포넌트로 추가하세요.
public class UnityClient_Sample : MonoBehaviour
{
    // 윈도우 서버 IP 주소를 입력하세요 (예: "ws://192.168.0.30:5000/ws")
    [SerializeField] private string serverUri = "ws://192.168.0.49/ws";
    
    // 움직일 대상 오브젝트 (비워두면 이 스크립트가 붙은 오브젝트가 움직임)
    public Transform targetObject;

    // 메세지를 화면에 띄울 TextMeshPro 컴포넌트
    [SerializeField] private TMP_Text displayMessageText;

    private ClientWebSocket _cws;
    private CancellationTokenSource _cts;

    // 움직임 데이터 저장용
    private Vector3 _currentLinear = Vector3.zero;
    private Vector3 _currentAngular = Vector3.zero;

    private async void Start()
    {
        if (targetObject == null) targetObject = transform;

        _cws = new ClientWebSocket();
        _cts = new CancellationTokenSource();

        try
        {
            await _cws.ConnectAsync(new Uri(serverUri), _cts.Token);
            Debug.Log("Connected to ROS2 Server!");

            // 수신 루프 시작
            _ = ReceiveLoop();
        }
        catch (Exception e)
        {
            Debug.LogError($"Connection failed: {e.Message}");
        }
    }

    private async Task ReceiveLoop()
    {
        var buffer = new byte[1024 * 4];

        while (_cws.State == WebSocketState.Open && !_cts.IsCancellationRequested)
        {
            try
            {
                var result = await _cws.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);
                
                if (result.MessageType == WebSocketMessageType.Text)
                {
                    var message = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    
                    // JSON 파싱 (메인 스레드에서 처리해도 되지만, 여기서는 데이터 파싱만)
                    TwistData data = JsonUtility.FromJson<TwistData>(message);
                    
                    // 값 갱신
                    _currentLinear = new Vector3(data.linear.x, data.linear.y, data.linear.z);
                    _currentAngular = new Vector3(data.angular.x, data.angular.y, data.angular.z);

                    // 화면 및 콘솔에 메세지 출력
                    UpdateStatusUI(message, data);
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Receive Error: {ex.Message}");
                break;
            }
        }
    }

    private void Update()
    {
        // ROS2 좌표계(X:앞)를 유니티 좌표계(Z:앞)로 변환해서 적용해야 할 수 있음.
        // 여기서는 단순하게 ROS2 Linear.x를 유니티 Z축(전진) 이동으로 매핑합니다.
        
        if (targetObject != null)
        {
            // 전진/후진 (Linear X -> Translate Z)
            targetObject.Translate(Vector3.forward * _currentLinear.x * Time.deltaTime);

            // 회전 (Angular Z -> Rotate Y)
            // ROS2 좌표계에서 Z축 회전은 유니티의 Y축 회전(Yaw)에 해당합니다.
            // 방향(부호)은 로봇과 유니티 설정에 따라 반대일 수 있으므로 테스트 필요.
            targetObject.Rotate(Vector3.up * -_currentAngular.z * 180 / Mathf.PI * Time.deltaTime);
        }
    }

    private void UpdateStatusUI(string rawJson, TwistData data)
    {
        // 콘솔에 디버그 로그 출력
        Debug.Log($"[ROS2 Received]: {rawJson}");

        // UI TextMeshPro에 정보 표시
        if (displayMessageText != null)
        {
            string status = $"<b>ROS2 Twist Data</b>\n" +
                            $"Linear X: {data.linear.x:F2}\n" +
                            $"Angular Z: {data.angular.z:F2}\n" +
                            $"<size=80%>Raw: {rawJson}</size>";
            displayMessageText.text = status;
        }
    }

    private void OnDestroy()
    {
        _cts?.Cancel();
        _cws?.Dispose();
    }

    // JSON 파싱을 위한 데이터 클래스
    [Serializable]
    public class TwistData
    {
        public Vector3Data linear;
        public Vector3Data angular;
    }

    [Serializable]
    public class Vector3Data
    {
        public float x;
        public float y;
        public float z;
    }
}

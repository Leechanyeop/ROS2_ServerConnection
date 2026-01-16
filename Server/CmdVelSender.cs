using System;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace Server
{
    /// <summary>
    /// SBC(192.168.0.21:22)로 임의의 Cmd_vel 값을 전송하는 클래스입니다.
    /// </summary>
    public class CmdVelSender
    {
        private const string SbcIp = "192.168.0.21";
        private const int SbcPort = 9090; // 주의: 22번 포트는 일반적으로 SSH용입니다. 
                                        // SBC 측에서 해당 포트로 TCP 서버가 실행 중이어야 합니다.

        public static async Task StartSendingAsync(CancellationToken ct)
        {
            Random random = new Random();
            Console.WriteLine($"[SBC Sender] {SbcIp}:{SbcPort}로 데이터 전송 루프 시작...");

            while (!ct.IsCancellationRequested)
            {
                try
                {
                    using (var client = new TcpClient())
                    {
                        // 연결 시도 (3초 타임아웃)
                        var connectTask = client.ConnectAsync(SbcIp, SbcPort, ct).AsTask();
                        if (await Task.WhenAny(connectTask, Task.Delay(3000, ct)) == connectTask)
                        {
                            await connectTask; // 연결 완료

                            using (var stream = client.GetStream())
                            {
                                // 1. 임의의 Cmd_vel 데이터 생성 (ROS2 Twist 메시지 규격)
                                var twistData = new
                                {
                                    linear = new { 
                                        x = Math.Round(random.NextDouble() * 2.0 - 1.0, 2), // -1.0 ~ 1.0
                                        y = 0.0, 
                                        z = 0.0 
                                    },
                                    angular = new { 
                                        x = 0.0, 
                                        y = 0.0, 
                                        z = Math.Round(random.NextDouble() * 2.0 - 1.0, 2)  // -1.0 ~ 1.0
                                    }
                                };

                                // 2. JSON 직렬화
                                string json = JsonSerializer.Serialize(twistData);
                                byte[] bytes = Encoding.UTF8.GetBytes(json + "\n"); // 개행문자 추가 (수신측 파싱 용이)

                                // 3. 전송
                                await stream.WriteAsync(bytes, 0, bytes.Length, ct);
                                Console.WriteLine($"[SBC Sent] {json}");
                            }
                        }
                        else
                        {
                            Console.WriteLine($"[SBC Timeout] {SbcIp}:{SbcPort} 연결 실패 (타임아웃)");
                        }
                    }
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[SBC Error] {ex.Message}");
                }

                // 1초 간격 전송
                await Task.Delay(1000, ct);
            }
        }
    }
}

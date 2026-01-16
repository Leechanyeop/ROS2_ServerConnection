using System.Net.WebSockets;
using System.Text.Json;
using System.Collections.Concurrent;

var builder = WebApplication.CreateBuilder(args);
builder.WebHost.UseUrls("http://0.0.0.0:5000"); // 모든 IP에서 접속 허용
var app = builder.Build();

app.UseWebSockets();

// 연결된 모든 클라이언트를 관리하기 위한 스레드 안전한 리스트
var connectedClients = new ConcurrentDictionary<WebSocket, byte>();

// [추가] SBC로 임의의 Cmd_vel을 보내는 백그라운드 태스크 시작
var cts = new CancellationTokenSource();
_ = Server.CmdVelSender.StartSendingAsync(cts.Token);

app.Map("/ws", async context => {
    if (context.WebSockets.IsWebSocketRequest) {
        using var ws = await context.WebSockets.AcceptWebSocketAsync();
        connectedClients.TryAdd(ws, 0); // 클라이언트 추가
        Console.WriteLine($"Client connected. Total clients: {connectedClients.Count}");

        var buffer = new byte[1024 * 4];
        int messageCount = 0;

        try {
            while (ws.State == WebSocketState.Open) {
                var result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);
                
                if (result.MessageType == WebSocketMessageType.Text) {
                    var message = System.Text.Encoding.UTF8.GetString(buffer, 0, result.Count);
                    // Console.WriteLine($"Received: {message}"); 

                    // [Broadcast] 받은 메시지를 다른 모든 클라이언트(Unity 등)에게 전송
                    var responseBytes = System.Text.Encoding.UTF8.GetBytes(message);
                    var tasks = connectedClients.Keys.Select(async client => {
                        if (client != ws && client.State == WebSocketState.Open) {
                            await client.SendAsync(
                                new ArraySegment<byte>(responseBytes), 
                                WebSocketMessageType.Text, 
                                true, 
                                CancellationToken.None);
                        }
                    });
                    await Task.WhenAll(tasks);

                    try {
                        // JSON Parsing (유효성 검사 목적)
                        var cmd = JsonSerializer.Deserialize<JsonElement>(message);
                        messageCount++;
                        
                        if (messageCount % 50 == 0) {
                            Console.WriteLine($"Server Status: Processed {messageCount} messages.");
                        }
                    } catch (Exception ex) {
                        Console.WriteLine($"Error parsing JSON: {ex.Message}");
                    }
                } else if (result.MessageType == WebSocketMessageType.Close) {
                    await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Closing", CancellationToken.None);
                }
            }
        } finally {
            connectedClients.TryRemove(ws, out _); // 클라이언트 제거
            Console.WriteLine($"Client disconnected. Remaining: {connectedClients.Count}");
        }
    } else {
        context.Response.StatusCode = StatusCodes.Status400BadRequest;
    }
});

app.Lifetime.ApplicationStopping.Register(() => cts.Cancel());

app.Run();

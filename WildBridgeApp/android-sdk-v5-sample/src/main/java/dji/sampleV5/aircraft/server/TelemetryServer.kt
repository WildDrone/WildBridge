package dji.sampleV5.aircraft.server

import android.util.Log
import java.io.PrintWriter
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.Executors
import kotlin.concurrent.thread

class TelemetryServer(
    private val port: Int,
    private val telemetryProvider: () -> String
) {
    private var serverSocket: ServerSocket? = null
    private val executor = Executors.newCachedThreadPool()
    @Volatile
    private var isRunning = false
    private var serverThread: Thread? = null
    private val clients = ConcurrentHashMap<Socket, PrintWriter>()

    /** Callback invoked (once) when the first client connects. Receives the client's IP address. */
    var onFirstClientConnected: ((clientIp: String) -> Unit)? = null
    @Volatile
    private var firstClientNotified = false

    fun start() {
        if (isRunning) return

        serverThread = thread(name = "TelemetryServer-$port", start = true) {
            try {
                serverSocket = ServerSocket(port)
                isRunning = true
                Log.i("TelemetryServer", "Server started on port $port")

                // Start a thread to periodically send telemetry data to all connected clients
                executor.submit { sendTelemetryData() }

                while (isRunning && !serverSocket!!.isClosed) {
                    try {
                        val clientSocket = serverSocket!!.accept()
                        val clientIp = clientSocket.inetAddress.hostAddress ?: "unknown"
                        Log.i("TelemetryServer", "Client connected: $clientIp")
                        val writer = PrintWriter(clientSocket.getOutputStream(), true)
                        clients[clientSocket] = writer
                        // Notify once when the first client (bridge) connects
                        if (!firstClientNotified) {
                            firstClientNotified = true
                            onFirstClientConnected?.invoke(clientIp)
                        }
                    } catch (e: Exception) {
                        if (isRunning) {
                            Log.e("TelemetryServer", "Error accepting connection: ${e.message}")
                        }
                    }
                }
            } catch (e: Exception) {
                Log.e("TelemetryServer", "Server error: ${e.message}")
            }
        }
    }

    private fun sendTelemetryData() {
        while (isRunning) {
            try {
                val telemetryJson = telemetryProvider()
                val clientsToRemove = mutableListOf<Socket>()

                for ((socket, writer) in clients) {
                    if (socket.isClosed || !socket.isConnected) {
                        clientsToRemove.add(socket)
                        continue
                    }
                    try {
                        writer.println(telemetryJson)
                        if (writer.checkError()) {
                            // Error occurred, likely client disconnected
                            clientsToRemove.add(socket)
                        }
                    } catch (e: Exception) {
                        Log.e("TelemetryServer", "Error sending data to client: ${e.message}")
                        clientsToRemove.add(socket)
                    }
                }

                // Remove disconnected clients
                clientsToRemove.forEach { socket ->
                    try {
                        socket.close()
                    } catch (e: Exception) {
                        // Ignore
                    }
                    clients.remove(socket)
                    Log.i("TelemetryServer", "Client disconnected and removed.")
                }

                Thread.sleep(10) // Send data at ~100Hz (cache rebuilt by SDK listeners)
            } catch (e: Exception) {
                Log.e("TelemetryServer", "Error in telemetry sending loop: ${e.message}")
            }
        }
    }

    fun stop() {
        isRunning = false
        onFirstClientConnected = null
        try {
            clients.values.forEach { it.close() }
            clients.keys.forEach { it.close() }
            clients.clear()
            serverSocket?.close()
            serverSocket = null
            executor.shutdownNow()
            if (Thread.currentThread() != serverThread) {
                serverThread?.join(1000)
            }
            serverThread = null
        } catch (e: Exception) {
            Log.e("TelemetryServer", "Error stopping server: ${e.message}")
        }
    }
}


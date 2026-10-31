import asyncio

import grpc

from argus.protocol.v1 import link_pb2, link_pb2_grpc, node_messages_pb2


class EchoHub(link_pb2_grpc.HubLinkServicer):
    async def Session(self, request_iterator, context):
        seq = 0
        async for msg in request_iterator:
            seq += 1
            out = link_pb2.HubMessage(seq=seq)
            if msg.HasField("hello"):
                out.session_accept.session_id = "s-" + msg.hello.node_id
            elif msg.HasField("heartbeat"):
                out.heartbeat.CopyFrom(msg.heartbeat)
            else:
                continue
            yield out

    async def Sensors(self, request_iterator, context):
        count = 0
        async for _ in request_iterator:
            count += 1
        return link_pb2.SensorAck(frames_received=count)


async def _serve():
    server = grpc.aio.server()
    link_pb2_grpc.add_HubLinkServicer_to_server(EchoHub(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    await server.start()
    return server, port


def test_session_stream_carries_both_directions():
    async def run():
        server, port = await _serve()
        try:
            async with grpc.aio.insecure_channel(f"127.0.0.1:{port}") as channel:
                call = link_pb2_grpc.HubLinkStub(channel).Session()
                hello = link_pb2.NodeMessage(seq=1)
                hello.hello.node_id = "argo-1"
                await call.write(hello)
                reply = await call.read()
                assert reply.session_accept.session_id == "s-argo-1"

                beat = link_pb2.NodeMessage(seq=2)
                beat.heartbeat.seq = 9
                await call.write(beat)
                reply = await call.read()
                assert reply.WhichOneof("body") == "heartbeat"
                assert reply.heartbeat.seq == 9
                await call.done_writing()
                assert await call.code() == grpc.StatusCode.OK
        finally:
            await server.stop(None)

    asyncio.run(run())


def test_sensor_stream_is_separate():
    async def run():
        server, port = await _serve()
        try:
            async with grpc.aio.insecure_channel(f"127.0.0.1:{port}") as channel:
                frames = (node_messages_pb2.SensorFrame(sensor_id="front") for _ in range(5))
                ack = await link_pb2_grpc.HubLinkStub(channel).Sensors(frames)
                assert ack.frames_received == 5
        finally:
            await server.stop(None)

    asyncio.run(run())

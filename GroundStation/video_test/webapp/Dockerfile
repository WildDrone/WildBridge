FROM ros:humble-ros-base

WORKDIR /app

COPY server.py ./server.py
COPY public ./public

ENV PORT=8090

CMD ["python3", "server.py"]

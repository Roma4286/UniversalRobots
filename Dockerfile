FROM python:3.11-slim

WORKDIR /app
COPY . /app

RUN pip install ur-rtde

CMD ["python", "ur_rtde.py"]

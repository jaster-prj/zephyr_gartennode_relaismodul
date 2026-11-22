FROM python:3.13-slim

WORKDIR /app
COPY testbed/companion/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt
COPY testbed/companion/companion.py /app/companion.py

CMD ["python", "-u", "/app/companion.py"]

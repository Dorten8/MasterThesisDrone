import inspect
from mcap.reader import NonSeekingReader

print("Printing source of NonSeekingReader:")
try:
    print(inspect.getsource(NonSeekingReader))
except Exception as e:
    print(f"Failed to inspect: {e}")

from transformers import GroundingDinoProcessor
from transformers import GroundingDinoForObjectDetection
import torch
import matplotlib.pyplot as plt


def preprocess_caption(caption: str) -> str:
    result = caption.lower().strip()
    if result.endswith("."):
        return result
    return result + "."


def _detect_object(text):
    processor = GroundingDinoProcessor.from_pretrained("IDEA-Research/grounding-dino-base")
    text = "a cat"
    inputs = processor(images=image, text=preprocess_caption(text), return_tensors="pt")
    model = GroundingDinoForObjectDetection.from_pretrained("IDEA-Research/grounding-dino-base")
    with torch.no_grad():
        outputs = model(**inputs)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import soundfile as sf
import numpy as np

try:
    import sherpa_onnx
except ImportError:
    print("Error: sherpa_onnx not installed. Run: pip3 install sherpa-onnx")
    sys.exit(1)

# Configuration
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_ROOT = os.path.join(CURRENT_DIR, "voice_models", "tts_models")
ZH_MODEL_DIR = os.path.join(MODELS_ROOT, "vits-melo-tts-zh_en")

ZH_CONFIG = {
    "model": os.path.join(ZH_MODEL_DIR, "model.onnx"),
    "tokens": os.path.join(ZH_MODEL_DIR, "tokens.txt"),
    "lexicon": os.path.join(ZH_MODEL_DIR, "lexicon.txt"),
    "dict_dir": os.path.join(ZH_MODEL_DIR, "dict"),
    "data_dir": "" 
}

OUTPUT_WAV = "test_zh_output.wav"

def main():
    print("="*40)
    print("       MeloTTS (ZH) Test Script")
    print("="*40)
    
    print(f"Checking model path: {ZH_CONFIG['model']}")
    if not os.path.exists(ZH_CONFIG['model']):
        print(f"Error: Model file not found at {ZH_CONFIG['model']}")
        return

    print("Initializing MeloTTS (ZH/Mix) model...")
    try:
        vits_config = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=ZH_CONFIG["model"],
            tokens=ZH_CONFIG["tokens"],
            data_dir=ZH_CONFIG["data_dir"],
        )
        
        if os.path.exists(ZH_CONFIG["lexicon"]):
            vits_config.lexicon = ZH_CONFIG["lexicon"]
        if os.path.exists(ZH_CONFIG["dict_dir"]):
            vits_config.dict_dir = ZH_CONFIG["dict_dir"]

        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=vits_config,
                provider="cpu",
                num_threads=1,
                debug=False,
            ),
            rule_fsts="",
            max_num_sentences=1,
        )
        tts = sherpa_onnx.OfflineTts(config)
        print("Model initialized successfully.")
    except Exception as e:
        print(f"Failed to initialize model: {e}")
        return

    text = "这是一个测试脚本，只使用 .M.e.l.o.T.T.S 中文模型进行语音合成。Testing English words inside Chinese sentence."
    print(f"\nGenerating audio for: '{text}'")
    
    start_t = time.time()
    audio = tts.generate(text, sid=0, speed=1.0)
    end_t = time.time()
    
    if len(audio.samples) > 0:
        print(f"Generated {len(audio.samples)} samples in {end_t - start_t:.2f}s")
        sf.write(OUTPUT_WAV, audio.samples, audio.sample_rate)
        print(f"Saved to {OUTPUT_WAV}")
        
        # Try to play
        print("Playing...")
        os.system(f"aplay -q {OUTPUT_WAV}")
    else:
        print("Error: No audio generated.")

if __name__ == "__main__":
    main()

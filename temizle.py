import os
import sys
import platform

def kill_all_python():
    print("🧹 Temizlik Başlatılıyor...")
    
    system_os = platform.system()
    
    try:
        if system_os == "Windows":
            # Windows için zorla kapatma komutu
            # /F = Force (Zorla), /IM = Image Name (Dosya adı)
            print("WINDOWS üzerinde Python işlemleri sonlandırılıyor...")
            ret = os.system("taskkill /IM python.exe /F")
            
            if ret == 0:
                print("✅ Tüm Python işlemleri başarıyla kapatıldı.")
            elif ret == 128:
                print("ℹ️ Zaten çalışan bir Python işlemi bulunamadı (Temiz).")
            else:
                print("⚠️ Bir şeyler ters gitti veya işlem bulunamadı.")
                
        else:
            # Linux / Mac için
            print("LINUX/MAC üzerinde Python işlemleri sonlandırılıyor...")
            os.system("pkill -9 python")
            os.system("pkill -9 python3")
            print("✅ Temizlik komutu gönderildi.")

    except Exception as e:
        print(f"❌ Hata oluştu: {e}")

    print("------------------------------------------------")
    print("🚀 Portlar serbest bırakıldı. Sistemi yeniden başlatabilirsin.")

if __name__ == "__main__":
    kill_all_python()
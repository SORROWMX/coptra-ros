#!/bin/bash

# Скрипт для сборки образа Orange Pi 3B на виртуальной машине
# Использование: ./build-on-vm.sh

set -e

echo "🚀 Запуск сборки образа Orange Pi 3B Coptra на VM"
echo "=================================================="

# Проверка прав root
if [ "$EUID" -ne 0 ]; then
    echo "❌ Запустите скрипт с правами root: sudo ./build-on-vm.sh"
    exit 1
fi

# Проверка доступности необходимых команд
echo "🔍 Проверка зависимостей..."
for cmd in qemu-aarch64-static kpartx parted unzip wget curl git; do
    if ! command -v $cmd &> /dev/null; then
        echo "❌ Команда $cmd не найдена. Установите зависимости:"
        echo "   apt install -y qemu-user-static kpartx parted unzip wget curl git"
        exit 1
    fi
done

# Проверка binfmt
if ! update-binfmts --status | grep -q "qemu-aarch64"; then
    echo "⚠️  Настройка binfmt для QEMU..."
    update-binfmts --enable qemu-aarch64
fi

# Настройка переменных окружения
export DEBIAN_FRONTEND=noninteractive
export TZ=UTC
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# Проверка свободного места
AVAILABLE_SPACE=$(df . | tail -1 | awk '{print $4}')
if [ "$AVAILABLE_SPACE" -lt 10485760 ]; then  # 10GB в KB
    echo "❌ Недостаточно свободного места. Нужно минимум 10GB"
    echo "   Доступно: $((AVAILABLE_SPACE / 1024 / 1024))GB"
    exit 1
fi

echo "✅ Зависимости проверены"
echo "💾 Свободное место: $((AVAILABLE_SPACE / 1024 / 1024))GB"

# Создание директории для образов если не существует
mkdir -p images

# Очистка старых loop устройств
echo "🧹 Очистка старых loop устройств..."
losetup -D 2>/dev/null || true

# Сделать скрипты исполняемыми
echo "🔧 Настройка скриптов..."
chmod +x builder/*.sh

# Запуск сборки
echo "🏗️  Начинаем сборку образа..."
echo "   Это может занять 1-3 часа в зависимости от мощности VM"
echo ""

# Запуск основного скрипта сборки
./builder/image-build.sh

# Проверка результата
if [ $? -eq 0 ]; then
    echo ""
    echo "🎉 Сборка завершена успешно!"
    echo "📁 Образ находится в папке images/"
    ls -la images/*.img 2>/dev/null || echo "   Образы не найдены"
    
    # Предложение сжать образ
    echo ""
    read -p "🗜️  Сжать образ для экономии места? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🗜️  Сжимаем образ..."
        for img in images/*.img; do
            if [ -f "$img" ]; then
                echo "   Сжимаем $img..."
                xz -9 "$img"
                echo "   ✅ Готово: ${img}.xz"
            fi
        done
    fi
    
    echo ""
    echo "🎯 Следующие шаги:"
    echo "   1. Скачайте образ с VM"
    echo "   2. Запишите на microSD карту с помощью balenaEtcher"
    echo "   3. Вставьте карту в Orange Pi 3B и включите"
    echo "   4. Подключитесь к WiFi 'coptra-XXXX' (пароль: coptrawifi)"
    echo "   5. Откройте http://192.168.11.1 в браузере"
    
else
    echo ""
    echo "❌ Сборка завершилась с ошибкой!"
    echo "🔍 Проверьте логи выше для диагностики"
    exit 1
fi

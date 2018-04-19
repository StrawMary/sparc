
# exemplu de rulat scriptul: ./facenet_usage.sh to_align_dir permanent_aligned_faces_dir ./classifiers/permanentClassifier.pkl

# scriptul asta va primi un folder cu poze ce trebuie aliniate (ale persoanei care trebuie adaugata in clasificator)
# mai primeste si pathul catre celelalte foldere aliniate 
# va mai primi si pathul spre unde sa copieze clasificatorul creat de facenet
# aliniaza pozele din primul director, le copiaza pe celelalte langa si reface clasificatorul
# copiaza calsificatorul unde trebuie
# copiaza si noile fete aliniate unde trebuie

# primul si al doile parametru
to_align_dir=$1
aligned_images_dir=$2
classifier_path=$3

# fac curatenie in folderele din facenet daca exista ceva (folderul input/output/clasificator)
input_dir="./facenet/input_dir/"
out_dir="./facenet/out_dir/" # asta nu cred ca imi trebuie
my_class_dir="./facenet/my_class/"
# accesez variabilele create: echo $myvariable $anothervar
rm -r $input_dir*
rm -r $out_dir*
rm -r $my_class_dir*

# copiaza folderul cu pozele persoanei de aliniat in cel input_dir
cp -r $to_align_dir/* $input_dir


# rulez align pe folderul ala
cd ./facenet
python aligndata_first.py
cd ..

# sterg fisierul txt ce se creaza dupaalignment
rm $out_dir/bounding_boxes*

# acum out dir are doar un folder cu pozele aliniate ale noii persoane
# le copiez in fisierul de unde o sa le iau pe toate
cp -r $out_dir* $aligned_images_dir/

# mut toate fetele aliniate in out_dir
cp -r $aligned_images_dir/* $out_dir

# fac clasificare pe toate, adica rulez scriptule ca sa obtin noul clasificator
cd ./facenet
python create_classifier_se.py
cd ..

# redenumesc fisierul cu clasificatorul si copiez clasificatorul obtinut unde trebuie
mv $my_class_dir/my_classifier.pkl $classifier_path 

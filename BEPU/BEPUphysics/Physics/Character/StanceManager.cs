﻿using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUphysics.CollisionTests;
using BEPUphysics.Settings;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Handles a cylindrical character's stances, like standing or crouching, and their transitions.
    /// </summary>
    public class StanceManager
    {
        /// <summary>
        /// This is a direct reference to the 'true' character collidable. The others are query proxies that share the same shape.
        /// </summary>
        private Cylinder characterBody;

        ConvexCollidable<CylinderShape> standingQueryObject;
        ConvexCollidable<CylinderShape> crouchingQueryObject;
        ConvexCollidable<CylinderShape> proneQueryObject;
        ConvexCollidable<CylinderShape> currentQueryObject;

        /// <summary>
        /// Updates the query objects to match the character controller's current state.  Called when BodyRadius, StanceManager.StandingHeight, or StanceManager.CrouchingHeight is set.
        /// </summary>
        public void UpdateQueryShapes()
        {
            standingQueryObject.Shape.Radius = characterBody.Radius;
            standingQueryObject.Shape.Height = StandingHeight;
            standingQueryObject.Shape.CollisionMargin = characterBody.CollisionInformation.Shape.CollisionMargin;
            crouchingQueryObject.Shape.Radius = characterBody.Radius;
            crouchingQueryObject.Shape.Height = CrouchingHeight;
            crouchingQueryObject.Shape.CollisionMargin = characterBody.CollisionInformation.Shape.CollisionMargin;
            proneQueryObject.Shape.Radius = characterBody.Radius;
            proneQueryObject.Shape.Height = ProneHeight;
            proneQueryObject.Shape.CollisionMargin = characterBody.CollisionInformation.Shape.CollisionMargin;
        }

        private float standingHeight;
        /// <summary>
        /// Gets or sets the height of the character while standing.  To avoid resizing-related problems, use this only when the character is not being actively simulated or is not currently standing.
        /// </summary>
        public float StandingHeight
        {
            get { return standingHeight; }
            set
            {
                if (value <= 0 || value < CrouchingHeight)
                    throw new ArgumentException("Standing height must be positive and greater than the crouching height.");
                standingHeight = value;
                UpdateQueryShapes();
                if (CurrentStance == Stance.Standing)
                {
                    //If we're currently standing, then the current shape must be modified as well.
                    //This isn't entirely safe, but dynamic resizing generally isn't.
                    characterBody.Height = standingHeight;
                }
            }
        }

        private float crouchingHeight;
        /// <summary>
        /// Gets or sets the height of the character while crouching.  Must be less than the standing height.  To avoid resizing-related problems, use this only when the character is not being actively simulated or is not currently crouching.
        /// </summary>
        public float CrouchingHeight
        {
            get { return crouchingHeight; }
            set
            {
                if (value <= 0 || value > StandingHeight)
                    throw new ArgumentException("Crouching height must be positive and less than the standing height.");
                crouchingHeight = value;
                UpdateQueryShapes();

                if (CurrentStance == Stance.Crouching)
                {
                    //If we're currently crouching, then the current shape must be modified as well.
                    //This isn't entirely safe, but dynamic resizing generally isn't.
                    characterBody.Height = crouchingHeight;
                }
            }
        }

        private float proneHeight;
        /// <summary>
        /// Gets or sets the height of the character while prone.  Must be less than the standing height.  To avoid resizing-related problems, use this only when the character is not being actively simulated or is not currently prone.
        /// </summary>
        public float ProneHeight
        {
            get { return proneHeight; }
            set
            {
                if (value <= 0 || value > CrouchingHeight)
                    throw new ArgumentException("Crouching height must be positive and less than the crouching height.");
                proneHeight = value;
                UpdateQueryShapes();

                if (CurrentStance == Stance.Prone)
                {
                    //If we're currently crouching, then the current shape must be modified as well.
                    //This isn't entirely safe, but dynamic resizing generally isn't.
                    characterBody.Height = proneHeight;
                }
            }
        }

        /// <summary>
        /// Gets the current stance of the character.
        /// </summary>
        public Stance CurrentStance
        {
            get;
            private set;
        }

        /// <summary>
        /// Gets or sets the stance that the character is trying to move into.
        /// </summary>
        public Stance DesiredStance
        {
            get;
            set;
        }

        private QueryManager QueryManager { get; set; }
        private SupportFinder SupportFinder { get; set; }

        /// <summary>
        /// Constructs a stance manager for a character.
        /// </summary>
        /// <param name="characterBody">The character's body entity.</param>
        /// <param name="crouchingHeight">Crouching height of the character.</param>
        /// <param name="proneHeight">Prone height of the character.</param>
        /// <param name="queryManager">Provider of queries used by the stance manager to test if it is okay to change stances.</param>
        /// <param name="supportFinder">Support finder used by the character.</param>
        public StanceManager(Cylinder characterBody, float crouchingHeight, float proneHeight, QueryManager queryManager, SupportFinder supportFinder)
        {
            this.QueryManager = queryManager;
            this.SupportFinder = supportFinder;
            this.characterBody = characterBody;
            standingHeight = characterBody.Height;
            if (crouchingHeight < standingHeight)
                this.crouchingHeight = crouchingHeight;
            else
                throw new ArgumentException("Crouching height must be less than standing height.");
            if (proneHeight < crouchingHeight)
                this.proneHeight = proneHeight;
            else
                throw new ArgumentException("Prone height must be less than crouching height.");

            //We can share the real shape with the query objects.
            currentQueryObject = new ConvexCollidable<CylinderShape>(characterBody.CollisionInformation.Shape);
            standingQueryObject = new ConvexCollidable<CylinderShape>(new CylinderShape(StandingHeight, characterBody.Radius) { CollisionMargin = currentQueryObject.Shape.CollisionMargin });
            crouchingQueryObject = new ConvexCollidable<CylinderShape>(new CylinderShape(CrouchingHeight, characterBody.Radius) { CollisionMargin = currentQueryObject.Shape.CollisionMargin });
            proneQueryObject = new ConvexCollidable<CylinderShape>(new CylinderShape(proneHeight, characterBody.Radius) { CollisionMargin = currentQueryObject.Shape.CollisionMargin });
            //Share the collision rules between the main body and its query objects.  That way, the character's queries return valid results.
            currentQueryObject.CollisionRules = characterBody.CollisionInformation.CollisionRules;
            standingQueryObject.CollisionRules = characterBody.CollisionInformation.CollisionRules;
            crouchingQueryObject.CollisionRules = characterBody.CollisionInformation.CollisionRules;
            proneQueryObject.CollisionRules = characterBody.CollisionInformation.CollisionRules;
        }

        private void PrepareQueryObject(EntityCollidable queryObject, ref Vector3 position)
        {
            RigidTransform transform;
            transform.Position = position;
            transform.Orientation = characterBody.Orientation;
            queryObject.UpdateBoundingBoxForTransform(ref transform, 0);
        }

        /// <summary>
        /// Checks if a transition from the current stance to the target stance is possible given the current environment.
        /// </summary>
        /// <param name="targetStance">Stance to check for transition safety.</param>
        /// <param name="newHeight">If the transition is safe, the new height of the character. Zero otherwise.</param>
        /// <param name="newPosition">If the transition is safe, the new location of the character body if the transition occurred. Zero vector otherwise.</param>
        /// <returns>True if the target stance is different than the current stance and the transition is valid, false otherwise.</returns>
        public bool CheckTransition(Stance targetStance, out float newHeight, out Vector3 newPosition)
        {
            var currentPosition = characterBody.position;
            var down = characterBody.orientationMatrix.Down;
            newPosition = new Vector3();
            newHeight = 0;

            if (CurrentStance != targetStance)
            {

                float currentHeight;
                switch (CurrentStance)
                {
                    case Stance.Prone:
                        currentHeight = proneHeight;
                        break;
                    case Stance.Crouching:
                        currentHeight = crouchingHeight;
                        break;
                    default:
                        currentHeight = standingHeight;
                        break;
                }
                float targetHeight;
                switch (targetStance)
                {
                    case Stance.Prone:
                        targetHeight = proneHeight;
                        break;
                    case Stance.Crouching:
                        targetHeight = crouchingHeight;
                        break;
                    default:
                        targetHeight = standingHeight;
                        break;
                }


                if (currentHeight >= targetHeight)
                {
                    //The character is getting smaller, so no validation queries are required.
                    if (SupportFinder.HasSupport)
                    {
                        //On the ground, so need to move the position down.
                        currentPosition.AddScaled( ref down, ((currentHeight - targetHeight) * 0.5f), out newPosition );
                    }
                    else
                    {
                        //We're in the air, so we don't have to change the position at all- just change the height.
                        newPosition = currentPosition;
                    }
                    newHeight = targetHeight;
                    return true;
                }
                //The character is getting bigger, so validation is required.
                ConvexCollidable<CylinderShape> queryObject;
                switch (targetStance)
                {
                    case Stance.Prone:
                        queryObject = proneQueryObject;
                        break;
                    case Stance.Crouching:
                        queryObject = crouchingQueryObject;
                        break;
                    default:
                        queryObject = standingQueryObject;
                        break;
                }

                var tractionContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                var supportContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                var sideContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                var headContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                try
                {
                    if (SupportFinder.HasSupport)
                    {
                        //Increasing in size requires a query to verify that the new state is safe.
                        //TODO: State queries can be expensive if the character is crouching beneath something really detailed.
                        //There are some situations where you may want to do an upwards-pointing ray cast first.  If it hits something,
                        //there's no need to do the full query.
                        currentPosition.AddScaled( ref down, -((targetHeight - currentHeight) * .5f), out newPosition);
                        PrepareQueryObject(queryObject, ref newPosition);
                        QueryManager.QueryContacts(queryObject, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);
                        if (IsObstructed(ref sideContacts, ref headContacts))
                        {
                            //Can't stand up if something is in the way!
                            return false;
                        }
                        newHeight = targetHeight;
                        return true;
                    }
                    else
                    {
                        //This is a complicated case.  We must perform a semi-downstep query.
                        //It's different than a downstep because the head may be obstructed as well.

                        float highestBound = 0;
                        float lowestBound = (targetHeight - currentHeight) * .5f;
                        float currentOffset = lowestBound;
                        float maximum = lowestBound;

                        int attempts = 0;
                        //Don't keep querying indefinitely.  If we fail to reach it in a few informed steps, it's probably not worth continuing.
                        //The bound size check prevents the system from continuing to search a meaninglessly tiny interval.
                        while (attempts++ < 5 && lowestBound - highestBound > Toolbox.BigEpsilon)
                        {
							Vector3 candidatePosition; currentPosition.AddScaled( ref down, currentOffset, out candidatePosition );
                            float hintOffset;
                            switch (TrySupportLocation(queryObject, ref candidatePosition, out hintOffset, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts))
                            {
                                case CharacterContactPositionState.Accepted:
                                    currentOffset += hintOffset;
                                    //Only use the new position location if the movement distance was the right size.
                                    if (currentOffset > 0 && currentOffset < maximum)
                                    {
                                        currentPosition.AddScaled( ref down, currentOffset, out newPosition );
                                        newHeight = targetHeight;
                                        return true;
                                    }
                                    else
                                    {
                                        return false;
                                    }
                                case CharacterContactPositionState.NoHit:
                                    highestBound = currentOffset + hintOffset;
                                    currentOffset = (lowestBound + highestBound) * .5f;
                                    break;
                                case CharacterContactPositionState.Obstructed:
                                    lowestBound = currentOffset;
                                    currentOffset = (highestBound + lowestBound) * .5f;
                                    break;
                                case CharacterContactPositionState.TooDeep:
                                    currentOffset += hintOffset;
                                    lowestBound = currentOffset;
                                    break;
                            }
                        }
                        //Couldn't find a hit.  Go ahead and get bigger!
                        newPosition = currentPosition;
                        newHeight = targetHeight;
                        return true;
                    }
                }
                finally
                {
                    tractionContacts.Dispose();
                    supportContacts.Dispose();
                    sideContacts.Dispose();
                    headContacts.Dispose();
                }
            }

            return false;
        }

        /// <summary>
        /// Attempts to change the stance of the character if possible.
        /// </summary>
        /// <returns>Whether or not the character was able to change its stance.</returns>
        public bool UpdateStance(out Vector3 newPosition)
        {
            float newHeight;
            if (CheckTransition(DesiredStance, out newHeight, out newPosition))
            {
                CurrentStance = DesiredStance;
                characterBody.Height = newHeight;
                return true;
            }
            return false;
        }

        bool IsObstructed(ref QuickList<CharacterContact> sideContacts, ref QuickList<CharacterContact> headContacts)
        {
            //No head contacts can exist!
            if (headContacts.Count > 0)
                return true;
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < sideContacts.Count; i++)
            {
                if (IsObstructive(ref sideContacts.Elements[i].Contact))
                    return true;
            }
            return false;
        }

        bool IsObstructive(ref ContactData contact)
        {
            //Can't stand up if there are new side contacts that are too deep.
            if (SupportFinder.SideContacts.Count == 0 && contact.PenetrationDepth > CollisionDetectionSettings.AllowedPenetration)
            {
                return true;
            }

            //Go through side-facing contact and check to see if the new contact is deeper than any existing contact in the direction of the existing contact.
            //This is equivalent to considering the existing contacts to define planes and then comparing the new contact against those planes.
            //Since we already have the penetration depths, we don't need to use the positions of the contacts.
            foreach (var c in SupportFinder.SideContacts)
            {
                float dot = c.Contact.Normal.Dot(ref contact.Normal );
                float depth = dot * c.Contact.PenetrationDepth;
                if (depth > Math.Max(c.Contact.PenetrationDepth, CollisionDetectionSettings.AllowedPenetration))
                    return true;

            }
            return false;
        }

        CharacterContactPositionState TrySupportLocation(ConvexCollidable<CylinderShape> queryObject, ref Vector3 position, out float hintOffset,
            ref QuickList<CharacterContact> tractionContacts, ref QuickList<CharacterContact> supportContacts, ref QuickList<CharacterContact> sideContacts, ref QuickList<CharacterContact> headContacts)
        {
            hintOffset = 0;
            PrepareQueryObject(queryObject, ref position);
            QueryManager.QueryContacts(queryObject, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);

            bool obstructed = IsObstructed(ref sideContacts, ref headContacts);
            if (obstructed)
            {
                return CharacterContactPositionState.Obstructed;
            }
            if (supportContacts.Count > 0)
            {
                CharacterContactPositionState supportState;
                CharacterContact supportContact;
                QueryManager.AnalyzeSupportState(ref tractionContacts, ref supportContacts, out supportState, out supportContact);
                var down = characterBody.orientationMatrix.Down;
                //Note that traction is not tested for; it isn't important for the stance manager.
                if (supportState == CharacterContactPositionState.Accepted)
                {
                    //We're done! The guess found a good spot to stand on.
                    //We need to have fairly good contacts after this process, so only push it up a bit.
                    hintOffset = Math.Min(0, Vector3.Dot(ref supportContact.Contact.Normal,ref  down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.Contact.PenetrationDepth));
                    return CharacterContactPositionState.Accepted;
                }
                else if (supportState == CharacterContactPositionState.TooDeep)
                {
                    //Looks like we have to keep trying, but at least we found a good hint.
                    hintOffset = Math.Min(0, Vector3.Dot(ref supportContact.Contact.Normal,ref  down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.Contact.PenetrationDepth));
                    return CharacterContactPositionState.TooDeep;
                }
                else //if (supportState == SupportState.Separated)
                {
                    //It's not obstructed, but the support isn't quite right.
                    //It's got a negative penetration depth.
                    //We can use that as a hint.
                    hintOffset = -.001f - Vector3.Dot(ref supportContact.Contact.Normal, ref down) * supportContact.Contact.PenetrationDepth;
                    return CharacterContactPositionState.NoHit;
                }
            }
            else //Not obstructed, but no support.
            {
                return CharacterContactPositionState.NoHit;
            }
        }



    }

    //The StanceManager, as is, is semi-extensible.  Technically additional states can be added.
    //However, there are parts that are hard coded for the sake of initial implementation simplicity,
    //so it won't be a cakewalk.
    /// <summary>
    /// Stance of a cylindrical character.
    /// </summary>
    public enum Stance
    {
        /// <summary>
        /// Tallest stance.
        /// </summary>
        Standing,
        /// <summary>
        /// Middle-height stance; must be taller than prone and shorter than standing.
        /// </summary>
        Crouching,
        /// <summary>
        /// The shortest stance; the character is essentially on the ground. Note that the width and length of the character do not change while prone.
        /// </summary>
        Prone
    }
}
